/*
 * Public API of the mouse-mover behavior, callable from other modules
 * (e.g. a soft-reset behavior) that need to stop the mover deterministically.
 */

#pragma once

void zmk_mouse_mover_force_off(void); /* used by soft-reset; no-op when off */
