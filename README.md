# zmk-mouse-mover

Toggleable, human-like mouse jiggler for [ZMK](https://zmk.dev) v0.3+.

Pressing the bound key toggles a movement generator on/off. While ON the
keyboard emits **only relative cursor motion** — no button clicks, no
keystrokes — in randomized, non-periodic gestures. Because the motion is
injected through the Zephyr input subsystem, ZMK's activity tracker counts
it: the central never idle-sleeps while the mover is on. No wake-lock APIs,
no polling while off.

## Usage

```dts
#include <behaviors/mouse_mover.dtsi>

/* enable the host route on the central build */
&mouse_mover_listener {
    status = "okay";
};
```

Bind `&mouse_mover 0` to any trigger (key, layer, leader sequence, combo):

```dts
jiggle_seq { sequence = <M M>; bindings = <&mouse_mover 0>; };
```

The parameter selects the movement profile (only `0` today).

## Movement model

Two nested randomized cycles, xorshift32 PRNG, integer-only:

1. **Dwell**: uniform 3–20 s between gestures; 1-in-8 dwells double
   ("look away"). Each gesture re-rolls everything, so there is no
   observable period.
2. **Gesture**: a signed 15–154 count displacement walked in 8
   bell-ramped micro-steps at 9–22 ms (roughly the trackpad's own 77 Hz),
   with 1-count line jitter. ~1 in 4 gestures is instead a single ±1–2
   count resting-hand twitch.

Sign re-rolls per gesture, so the cursor net-walk stays bounded on screen.

## Requirements & notes

- `CONFIG_ZMK_POINTING=y` (any board using pointing devices already has it).
- The module compiles **central-only**; a toggle triggered on a split
  peripheral is forwarded to the central (standard CENTRAL-locality
  behavior). The peripheral keeps sleeping normally.
- Works over USB and BLE (the listener route is transport-agnostic).
  Some hosts ignore HID mouse activity for unattended sleep policies —
  over USB is the reliable case.
- OFF state cost: one unscheduled work item and a ~40 B static struct.
  No timers, no event subscriptions, no radio traffic.
