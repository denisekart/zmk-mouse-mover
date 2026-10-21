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

Bind `&mouse_mover <profile>` to any trigger (key, layer, leader sequence,
combo); the parameter selects the movement profile:

```dts
mouse_mover_seq     { sequence = <M M>; bindings = <&mouse_mover 0>; }; /* straight */
mouse_mover_nat_seq { sequence = <M N>; bindings = <&mouse_mover 1>; }; /* natural curves */
```

Each press toggles the mover: off when running, otherwise on with that
profile. Pressing the *other* profile's binding while running switches
profiles live, without interrupting the in-flight gesture.

Every toggle raises `zmk_mouse_mover_state_changed` (`enabled`, `profile`)
so UIs (e.g. a status-screen indicator) can reflect the state.

## Movement model

1. **Dwell**: uniform 20–60 s between gestures; the first gesture fires
   ~0.5–1 s after enabling (always a full walk, never a twitch) so
   enabling is immediately visible. Each gesture re-rolls everything, so
   there is no observable period.
2. **Gesture**: a signed 40–420 count displacement per axis walked in 64
   bell-ramped micro-steps at 32–92 ms (with 0–3 ms jitter per step), so a
   gesture lasts 2–6 s with ease-in/out pacing and 1-count line jitter.
   On the straight profile, ~1 in 4 gestures is instead a single ±1–2
   count resting-hand twitch.
   On the curves profile (`1`) the path is an integer quadratic-Bézier
   approximation — a 2-segment polyline bowing 1–5/8 off the chord
   perpendicular, bent either way — sampled at the bell fraction.

Sign re-rolls per gesture, so the cursor net-walk stays bounded on screen.

While enabled, any key press (local or split-peripheral) or trackpad event
pauses the mover: the in-flight gesture finishes, then nothing moves until
**30 s** after the last user event, forced to at most **45 s** from the
start of the pause even under continuous typing. The mover's own mouse
reports do not count as user activity. While paused the module generates no
I2C/BLE/HID traffic beyond the trailing gesture.

## Requirements & notes

- `CONFIG_ZMK_POINTING=y` (any board using pointing devices already has it).
- The module compiles **central-only**; a toggle triggered on a split
  peripheral is forwarded to the central (standard CENTRAL-locality
  behavior). The peripheral keeps sleeping normally.
- Works over USB and BLE (the listener route is transport-agnostic).
  Some hosts ignore HID mouse activity for unattended sleep policies —
  over USB is the reliable case.
- OFF state cost: one unscheduled work item and a ~80 B static struct;
  the activity listeners piggyback on event/input dispatch ZMK runs anyway.
