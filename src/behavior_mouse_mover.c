#define DT_DRV_COMPAT zmk_behavior_mouse_mover

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/mouse_mover_state_changed.h>

LOG_MODULE_REGISTER(behavior_mouse_mover, CONFIG_ZMK_LOG_LEVEL);

#define PAUSE_RESUME_MS 30000 /* resume 30 s after the last user event */
#define PAUSE_MAX_MS 60000    /* forced resume after 60 s of silence */

/* Bell ramp for gesture micro-steps (quarter-wave, ease-in/out).
 * Prefix sums reach 256 so a step is (dist * p[i]) >> 8 with exact deltas
 * from the difference of consecutive prefixes. 24 entries keep deltas >= 1,
 * so every micro-step visibly moves the cursor over a 2-6 s gesture. */
static const uint16_t bell_prefix[] = {
    1,   4,   10,  17,  26,  37,  50,  64,  79,  95,  111, 128,
    145, 161, 177, 192, 206, 219, 230, 239, 246, 252, 255, 256};
#define BELL_STEPS ARRAY_SIZE(bell_prefix)

struct mouse_mover_data {
    const struct device *dev;
    bool enabled;
    bool paused;            /* user activity suppresses gestures */
    uint8_t profile;        /* 0 = straight, 1 = natural curves */
    uint32_t rng;
    int16_t dx;
    int16_t dy;
    int16_t sent_x;
    int16_t sent_y;
    /* Curve path: 2-segment polyline (0,0) -> arc_a -> arc_b sampled at the
     * bell fraction; unused by the straight profile. */
    int16_t arc_ax;
    int16_t arc_ay;
    int16_t arc_bx;
    int16_t arc_by;
    int8_t step; /* next micro-step index; -1 = dwelling between gestures */
    uint16_t step_ms;
    int64_t pause_started; /* uptime when the current pause began */
    int64_t resume_at;     /* earliest uptime the mover may gesture again */
    struct k_work_delayable work;
};

static struct mouse_mover_data *mm_instance; /* the single mover device */

static uint32_t mm_rand(struct mouse_mover_data *d) {
    d->rng ^= d->rng << 13;
    d->rng ^= d->rng >> 17;
    d->rng ^= d->rng << 5;
    return d->rng;
}

static int32_t mm_next_interval_ms(struct mouse_mover_data *d) {
    return 20000 + (int32_t)(mm_rand(d) % 40001); /* 20..60 s */
}

static void mm_start_gesture(struct mouse_mover_data *d) {
    uint32_t r = mm_rand(d);

    if (d->profile == 0 && (r & 3) == 3) {
        /* ~1 in 4: resting-hand twitch — one tiny nudge, no walk.
         * Magnitude from bit 4: bits 0-1 are pinned by this branch's guard. */
        d->dx = (int16_t)(1 + ((r >> 4) & 1)) * (((r >> 2) & 1) ? 1 : -1);
        d->dy = (int16_t)(1 + ((r >> 8) % 2)) * (((r >> 9) & 1) ? 1 : -1);
        d->step = BELL_STEPS - 1; /* single final step carries the delta */
    } else {
        /* 40..420 px per axis; sign bits 14/15 are disjoint from the
         * magnitude fields (bits 0-8, 3-11). */
        d->dx = (int16_t)(40 + (r % 381)) * (((r >> 14) & 1) ? 1 : -1);
        d->dy = (int16_t)(40 + ((r >> 3) % 381)) * (((r >> 15) & 1) ? 1 : -1);
        d->step = 0;
        if (d->profile == 1) {
            /* Integer quadratic-Bezier as a 2-segment polyline: bow the
             * chord midpoint 1..5 eighths off perpendicular, endpoint at
             * 3/4 chord + full bow so the path arcs then straightens.
             * |dy * kb| <= 420 * 5 fits int32; storage stays int16. */
            int kb = 1 + ((r >> 17) % 5);
            if ((r >> 20) & 1) {
                kb = -kb;
            }
            int32_t cx = (d->dx >> 1) + (((-((int32_t)d->dy)) * kb) >> 3);
            int32_t cy = (d->dy >> 1) + ((((int32_t)d->dx) * kb) >> 3);
            d->arc_ax = (int16_t)cx;
            d->arc_ay = (int16_t)cy;
            d->arc_bx = (int16_t)(cx + (d->dx >> 2));
            d->arc_by = (int16_t)(cy + (d->dy >> 2));
        }
    }
    d->sent_x = 0;
    d->sent_y = 0;
    d->step_ms = 85 + ((r >> 16) % 166); /* 85-250 ms: 2 s..6 s per gesture */
}

static void mm_work_fn(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mouse_mover_data *d = CONTAINER_OF(dwork, struct mouse_mover_data, work);
    int32_t rearm_ms;

    if (!d->enabled) {
        return;
    }

    if (d->step < 0) {
        if (d->paused) {
            int64_t remain = d->resume_at - k_uptime_get();
            if (remain > 0) {
                k_work_schedule(&d->work, K_MSEC(remain));
                return;
            }
            d->paused = false; /* silence long enough: gesture now */
        }
        mm_start_gesture(d);
        rearm_ms = d->step_ms;
    } else {
        rearm_ms = d->step_ms + (int32_t)((mm_rand(d) >> 8) & 3);
    }

    /* One bell-shaped micro-step: target displacement minus everything sent
     * so far, scaled through the prefix ramp — no accumulated rounding. */
    int i = d->step;
    uint16_t p = bell_prefix[i];
    int16_t sx;
    int16_t sy;
    if (d->profile == 1) {
        /* Polyline sample at the bell fraction, 1/256 fixed point. The
         * final step's p=256 lands exactly on the endpoint. */
        int f = (p <= 128) ? (int)p * 2 : ((int)p - 128) * 2;
        int32_t px;
        int32_t py;
        if (p <= 128) {
            px = ((int32_t)d->arc_ax * f) >> 8;
            py = ((int32_t)d->arc_ay * f) >> 8;
        } else {
            px = d->arc_ax + ((((int32_t)d->arc_bx - d->arc_ax) * f) >> 8);
            py = d->arc_ay + ((((int32_t)d->arc_by - d->arc_ay) * f) >> 8);
        }
        sx = (int16_t)(px - d->sent_x);
        sy = (int16_t)(py - d->sent_y);
    } else {
        sx = (int16_t)((((int32_t)d->dx - d->sent_x) * p) >> 8);
        sy = (int16_t)((((int32_t)d->dy - d->sent_y) * p) >> 8);
    }

    /* 1-count jitter, ~1 in 4 per axis: breaks machine-perfect lines. */
    uint32_t j = mm_rand(d);
    if ((j & 3) == 0) {
        sx += ((j >> 2) & 1) ? 1 : -1;
    } else if ((j & 3) == 1) {
        sy += ((j >> 2) & 1) ? 1 : -1;
    }
    d->sent_x += sx;
    d->sent_y += sy;

    /* Sync flag rides the second axis: one HID report per micro-step. */
    input_report_rel(d->dev, INPUT_REL_X, sx, false, K_NO_WAIT);
    input_report_rel(d->dev, INPUT_REL_Y, sy, true, K_NO_WAIT);

    d->step++;
    if (d->step >= (int)BELL_STEPS) {
        d->step = -1;
        if (d->paused) {
            int64_t remain = d->resume_at - k_uptime_get();
            rearm_ms = (remain > 0) ? (int32_t)remain : 0;
        } else {
            rearm_ms = mm_next_interval_ms(d);
        }
    }
    k_work_schedule(&d->work, K_MSEC(rearm_ms));
}

/* Called from the ZMK event-manager thread (keys, incl. split peripheral
 * positions) and the input subsystem (trackpad proxy). Fields are
 * single-word stores; pause_started is written before paused so a reader
 * that sees paused=true also sees the timestamp. */
static void mm_note_user_activity(void) {
    struct mouse_mover_data *d = mm_instance;

    if (d == NULL || !d->enabled) {
        return;
    }
    int64_t now = k_uptime_get();
    if (!d->paused) {
        d->pause_started = now;
        d->paused = true;
    }
    int64_t want = now + PAUSE_RESUME_MS;
    int64_t cap = d->pause_started + PAUSE_MAX_MS;
    if (want > cap) {
        want = cap;
    }
    d->resume_at = want;
    if (d->step < 0) {
        k_work_reschedule(&d->work, K_MSEC(want - now));
    }
    /* else: let the in-flight gesture finish; mm_work_fn re-arms paused. */
}

static int mm_position_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        mm_note_user_activity();
    }
    return 0;
}

ZMK_LISTENER(mouse_mover_pause, mm_position_listener);
ZMK_SUBSCRIPTION(mouse_mover_pause, zmk_position_state_changed);

/* Split-peripheral trackpad events re-inject on the central through the
 * zmk,input-split proxy device; comparing against exactly that device
 * ignores this behavior's own reports (evt->dev == mover dev). If the
 * listener chain ever stops feeding the central's input subsystem, the
 * position listener above still covers keys. */
static void mm_input_listener(struct input_event *evt) {
    static const struct device *proxy =
        DEVICE_DT_GET_OR_NULL(DT_NODELABEL(trackpad_split));
    if (proxy != NULL && evt->dev == proxy) {
        mm_note_user_activity();
    }
}
INPUT_CALLBACK_DEFINE(NULL, mm_input_listener);

static void mm_set_enabled(struct mouse_mover_data *d, bool on, uint8_t profile) {
    d->enabled = on;
    d->profile = profile;
    LOG_INF("mouse mover %s (profile %u)", on ? "on" : "off", profile);
    raise_zmk_mouse_mover_state_changed(
        (struct zmk_mouse_mover_state_changed){.enabled = on});
}

static int mm_on_binding_pressed(struct zmk_behavior_binding *binding,
                                 struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mouse_mover_data *d = dev->data;
    ARG_UNUSED(event);

    if (d->enabled && binding->param != d->profile) {
        /* Other-profile press while running: switch curves live. */
        d->profile = binding->param;
        LOG_INF("mouse mover profile %u", binding->param);
        raise_zmk_mouse_mover_state_changed(
            (struct zmk_mouse_mover_state_changed){.enabled = true});
        return ZMK_BEHAVIOR_OPAQUE;
    }

    bool on = !d->enabled;
    if (on) {
        if (d->rng == 0) {
            d->rng = k_cycle_get_32() | 1;
        }
        d->step = -1;
        d->paused = false;
        k_work_schedule(&d->work, K_MSEC(mm_next_interval_ms(d)));
    } else {
        k_work_cancel_delayable(&d->work);
        d->paused = false;
    }
    mm_set_enabled(d, on, binding->param);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int mm_on_binding_released(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding);
    ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int mouse_mover_init(const struct device *dev) {
    struct mouse_mover_data *d = dev->data;

    d->dev = dev;
    mm_instance = d;
    k_work_init_delayable(&d->work, mm_work_fn);
    return 0;
}

static const struct behavior_driver_api mouse_mover_driver_api = {
    .binding_pressed = mm_on_binding_pressed,
    .binding_released = mm_on_binding_released,
};

#define MOUSE_MOVER_INST(n)                                                                        \
    static struct mouse_mover_data mouse_mover_data_##n;                                           \
    BEHAVIOR_DT_INST_DEFINE(n, mouse_mover_init, NULL, &mouse_mover_data_##n, NULL, POST_KERNEL,   \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &mouse_mover_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MOUSE_MOVER_INST)
