/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

struct zmk_mouse_mover_state_changed {
    bool enabled;
    uint8_t profile; /* movement profile while enabled: 0 straight, 1 curves */
};

ZMK_EVENT_DECLARE(zmk_mouse_mover_state_changed);
