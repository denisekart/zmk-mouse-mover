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
#include <zmk/mouse_mover.h>

LOG_MODULE_REGISTER(behavior_mouse_mover, CONFIG_ZMK_LOG_LEVEL);

#define PAUSE_RESUME_MS 30000 /* resume 30 s after the last user event */

/* Bell ramp for gesture micro-steps (quarter sine-ease, ease-in/out).
 * Prefix sums reach 256 so a step is (dist * p[i]) >> 8 with exact deltas
 * from the difference of consecutive prefixes. 96 entries keep deltas 1-4
 * and >= 1, so every micro-step visibly moves the cursor over a 2-6 s
 * gesture; the denser ramp removes the stepping the 64-entry table showed. */
static const uint16_t bell_prefix[] = {
    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,
   14,   16,   18,   20,   22,   24,   26,   28,   31,   34,   37,   40,
   43,   46,   49,   52,   55,   58,   61,   64,   68,   72,   76,   80,
   84,   88,   92,   96,  100,  104,  108,  112,  116,  120,  124,  128,
  132,  136,  140,  144,  148,  152,  156,  160,  164,  168,  172,  176,
  180,  184,  188,  192,  195,  198,  201,  204,  207,  210,  213,  216,
  219,  222,  225,  228,  230,  232,  234,  236,  238,  240,  242,  244,
  245,  246,  247,  248,  249,  250,  251,  252,  253,  254,  255,  256};
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
    int8_t step; /* micro-step index; -1 dwell, -2 forced walk, -3 twitched, walk pending */
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

/* Returns true when only the twitch nudge was emitted (walk pending). */
static bool mm_start_gesture(struct mouse_mover_data *d) {
    uint32_t r = mm_rand(d);

    if (d->profile == 0 && d->step == -1 && (r & 3) == 3) {
        /* ~1 in 4: resting-hand twitch — a tiny nudge, then a settle beat
         * before the walk (-3). Every burst ends in a real walk, so no
         * detector ever sees a twitch-only or consecutive-twitch pattern.
         * Magnitude from bit 4: bits 0-1 are pinned by this branch's guard. */
        int16_t tx = (int16_t)(1 + ((r >> 4) & 1)) * (((r >> 2) & 1) ? 1 : -1);
        int16_t ty = (int16_t)(1 + ((r >> 8) % 2)) * (((r >> 9) & 1) ? 1 : -1);
        input_report_rel(d->dev, INPUT_REL_X, tx, false, K_NO_WAIT);
        input_report_rel(d->dev, INPUT_REL_Y, ty, true, K_NO_WAIT);
        d->step = -3;
        d->step_ms = 800 + ((r >> 16) % 1601); /* 0.8-2.4 s settle beat */
        return true;
    }
    {
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
    d->step_ms = 21 + ((r >> 16) % 43); /* 21-63 ms: 95 intervals -> 2 s..6 s */
    return false;
}

static void mm_work_fn(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mouse_mover_data *d = CONTAINER_OF(dwork, struct mouse_mover_data, work);
    int32_t rearm_ms;

    if (!d->enabled) {
        return;
    }

    if (d->paused && d->step != -2) {
        d->step = -1; /* abort any in-flight walk / pending settle beat */
        int64_t remain = d->resume_at - k_uptime_get();
        if (remain > 0) {
            k_work_schedule(&d->work, K_MSEC(remain));
            return;
        }
        d->paused = false; /* silence long enough: gesture now */
    }

    if (d->step < 0) {
        if (mm_start_gesture(d)) {
            k_work_schedule(&d->work, K_MSEC(d->step_ms)); /* settle beat */
            return;
        }
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
        /* Walk end: the next firing re-checks `paused` via the preempt
         * block at the top of this function. */
        rearm_ms = mm_next_interval_ms(d);
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
    d->resume_at = now + PAUSE_RESUME_MS;
    if (d->step == -2) {
        return; /* enable confirmation is immune to pausing */
    }
    if (d->step < 0) {
        k_work_reschedule(&d->work, K_MSEC(PAUSE_RESUME_MS));
    }
    /* In-flight walks are preempted by mm_work_fn on its next firing. */
}

static int mm_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    /* Presses only: the releases of the enabling leader keys must not
     * swallow the confirmation gesture. */
    if (ev != NULL && ev->state) {
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
        (struct zmk_mouse_mover_state_changed){.enabled = on, .profile = profile});
}

void zmk_mouse_mover_force_off(void) {
    struct mouse_mover_data *d = mm_instance;

    if (d != NULL && d->enabled) {
        k_work_cancel_delayable(&d->work);
        d->paused = false;
        mm_set_enabled(d, false, d->profile);
    }
}

static int mm_on_binding_pressed(struct zmk_behavior_binding *binding,
                                 struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mouse_mover_data *d = dev->data;
    ARG_UNUSED(event);

    if (d->enabled && binding->param1 != d->profile) {
        /* Other-profile press while running: switch curves live. */
        d->profile = (uint8_t)binding->param1;
        LOG_INF("mouse mover profile %u", binding->param1);
        raise_zmk_mouse_mover_state_changed(
            (struct zmk_mouse_mover_state_changed){.enabled = true, .profile = binding->param1});
        return ZMK_BEHAVIOR_OPAQUE;
    }

    bool on = !d->enabled;
    if (on) {
        if (d->rng == 0) {
            d->rng = k_cycle_get_32() | 1;
        }
        d->step = -2; /* first gesture: full walk, never a twitch */
        d->paused = false;
        /* Confirmation gesture ~0.5-1 s after enabling, then normal dwell. */
        k_work_schedule(&d->work, K_MSEC(500 + (int32_t)(mm_rand(d) % 501)));
    } else {
        k_work_cancel_delayable(&d->work);
        d->paused = false;
    }
    mm_set_enabled(d, on, (uint8_t)binding->param1);
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
