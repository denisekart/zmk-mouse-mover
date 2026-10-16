#define DT_DRV_COMPAT zmk_behavior_mouse_mover

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

LOG_MODULE_REGISTER(behavior_mouse_mover, CONFIG_ZMK_LOG_LEVEL);

/* Bell ramp for gesture micro-steps (quarter-wave, ease-in/out).
 * Prefix sums reach 256 so a step is (dist * p[i]) >> 8 with exact deltas
 * from the difference of consecutive prefixes. */
static const uint16_t bell_prefix[] = {4, 26, 57, 103, 154, 200, 231, 256};
#define BELL_STEPS ARRAY_SIZE(bell_prefix)

struct mouse_mover_data {
    const struct device *dev;
    bool enabled;
    uint32_t rng;
    int16_t dx;
    int16_t dy;
    int16_t sent_x;
    int16_t sent_y;
    int8_t step; /* next micro-step index; -1 = dwelling between gestures */
    uint16_t step_ms;
    struct k_work_delayable work;
};

static uint32_t mm_rand(struct mouse_mover_data *d) {
    d->rng ^= d->rng << 13;
    d->rng ^= d->rng >> 17;
    d->rng ^= d->rng << 5;
    return d->rng;
}

static int32_t mm_next_interval_ms(struct mouse_mover_data *d) {
    uint32_t r = mm_rand(d);
    int32_t dwell = 3000 + (r % 17000); /* 3..20 s */
    if (((r >> 16) & 7) == 0) {
        dwell *= 2; /* asynchronous "look away" pause */
    }
    return dwell;
}

static void mm_start_gesture(struct mouse_mover_data *d) {
    uint32_t r = mm_rand(d);

    if ((r & 3) == 3) {
        /* ~1 in 4: resting-hand twitch — one tiny nudge, no walk.
         * Magnitude from bit 4: bits 0-1 are pinned by this branch's guard. */
        d->dx = (int16_t)(1 + ((r >> 4) & 1)) * (((r >> 2) & 1) ? 1 : -1);
        d->dy = (int16_t)(1 + ((r >> 8) % 2)) * (((r >> 9) & 1) ? 1 : -1);
        d->step = BELL_STEPS - 1; /* single final step carries the delta */
    } else {
        d->dx = (int16_t)(15 + (r % 140)) * (((r >> 7) & 1) ? 1 : -1);
        d->dy = (int16_t)(15 + ((r >> 3) % 140)) * (((r >> 11) & 1) ? 1 : -1);
        d->step = 0;
    }
    d->sent_x = 0;
    d->sent_y = 0;
    d->step_ms = 9 + ((r >> 16) % 12); /* ~77 Hz, like the trackpad itself */
}

static void mm_work_fn(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct mouse_mover_data *d = CONTAINER_OF(dwork, struct mouse_mover_data, work);
    int32_t rearm_ms;

    if (!d->enabled) {
        return;
    }

    if (d->step < 0) {
        mm_start_gesture(d);
        rearm_ms = d->step_ms;
    } else {
        rearm_ms = d->step_ms + (int32_t)((mm_rand(d) >> 8) & 3);
    }

    /* One bell-shaped micro-step: target displacement minus everything sent
     * so far, scaled through the prefix ramp — no accumulated rounding. */
    int i = d->step;
    uint16_t p = bell_prefix[i];
    int16_t sx = (int16_t)((((int32_t)d->dx - d->sent_x) * p) >> 8);
    int16_t sy = (int16_t)((((int32_t)d->dy - d->sent_y) * p) >> 8);

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
        rearm_ms = mm_next_interval_ms(d);
    }
    k_work_schedule(&d->work, K_MSEC(rearm_ms));
}

static int mm_on_binding_pressed(struct zmk_behavior_binding *binding,
                                 struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct mouse_mover_data *d = dev->data;
    ARG_UNUSED(event);

    d->enabled = !d->enabled;
    LOG_INF("mouse mover %s", d->enabled ? "on" : "off");

    if (d->enabled) {
        if (d->rng == 0) {
            d->rng = k_cycle_get_32() | 1;
        }
        d->step = -1;
        k_work_schedule(&d->work, K_MSEC(mm_next_interval_ms(d)));
    } else {
        k_work_cancel_delayable(&d->work);
    }
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
