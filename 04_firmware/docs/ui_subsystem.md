# UI subsystem

The UI subsystem provides local status and deliberate user requests on
the Waveshare 2-inch IPS LCD module. The display uses an ST7789V
controller, a 240 x 320 RGB565 panel, and four-wire SPI.

Portable UI code has no ESP-IDF, FreeRTOS, GPIO, SPI-host, or board-pin
dependencies.

## Current RadioMaster Pocket mapping

| Channel | Radio control | Current UI function |
| --- | --- | --- |
| CH1 | Ail, right stick left/right | Previous/next page in browse mode |
| CH2 | Thr, right stick up/down | Page selection; velocity on CAN; current on CONTROL |
| CH3 | Ele, left stick up/down | Unassigned |
| CH4 | Rud, left stick left/right | Unassigned |
| CH5 | SA, latching switch | Unassigned |
| CH6 | SD, latching switch | Interact mode; requests the active commissioning mode |
| CH7 | SE, momentary switch | Enter, currently unused on CAN |
| CH8 | S1, non-return potentiometer | Reserved for future value adjustment |
| CH9 | SB, three-position switch | Unassigned |
| CH10 | SC, three-position switch | UI input lock/enable |

Channel numbers in `config_ui.h` are one-based so they match the EdgeTX
channel monitor.

## Input lock

SC must be in its high position before the UI accepts navigation,
interaction, or Enter events.

```text
SC low or middle
    Lock UI input.
    Return to browse mode.
    Ignore CH1, CH2, CH6, and CH7.

SC high
    Enable UI input after an intentional low/middle-to-high transition.
```

After boot or RC link recovery, SC must first be moved to low or middle
and then moved high. This prevents a transmitter that powers up with SC
already high from immediately enabling UI input.

When SC enables input:

```text
CH1 and CH2
    Must be neutral before an axis event can occur.

CH6
    Must be inactive before interact mode can be entered.

CH7
    Must be released before an Enter event can occur.
```

## UI state machine

```text
LOCKED
    SC high transition -> BROWSE

BROWSE
    CH1 left/right -> previous/next top-level page
    CH6 active -> INTERACT
    SC inactive or RC link lost -> LOCKED

INTERACT
    CH2 up/down -> previous/next selection on ordinary pages
    CAN page -> CH2 is the manual velocity input
    CONTROL page -> CH2 is the four-actuator current input
    CH7 press -> Enter event
    CH6 inactive -> BROWSE
    SC inactive or RC link lost -> LOCKED
```

Both stick axes require a return to neutral before another event. Holding
a stick does not auto-repeat. An interact-mode transition suppresses
simultaneous axis and Enter events.

Enter remains available for future page actions. It is not required for
manual drive.

## Current pages

```text
STATUS
    RC state and freshness.
    IMU state and freshness.
    Large roll and pitch values.

CRSF
    Selection 1 shows CH1-CH8.
    Selection 2 shows CH9-CH16.
    Shows link state, age, CRC errors, and parser errors.

IMU
    Selection 1 shows attitude, pitch rate, and calibration state.
    Selection 2 shows acceleration, gyro, temperature, and norm.

CONTROL
    Selection 1 shows the estimated position, attitude, velocity, and rates.
    Selection 2 shows state validity, loop counters, execution time, and faults.
    Selection 3 shows torque state, requested current, four approved currents,
    and the inhibit bit mask.
    SD requests torque-control mode; CH2 must be centered before LIVE.

CAN
    Shows CAN bus state and command transmit errors.
    Shows M1 ID, feedback state, position, electrical speed, current,
    temperature, and fault code.
    SD enables manual-drive request; SE is not required.
    Shows OFF, BLOCK, CENTER THR, LIVE, STOPPING, or SD OFF-ON status
    and the approved ERPM command.
```

The layout uses:

```text
Top bar
    Page name and LOCK/BROWSE/INTERACT state.

Content area
    Only page-relevant values.

Bottom bar
    Compact RC, IMU, and CAN health indicators.
    Page number and current selection number.
```

Permanent control instructions are intentionally omitted to reduce
clutter.

## Module responsibilities

```text
ui_rc_input
    Maps configured CRSF channels to semantic UI events.
    Owns input gating, thresholds, hysteresis, neutral re-arm, and
    reconnect behavior.

ui_state
    Owns locked/browse/interact behavior, page, selection, and Enter
    count.

ui_canvas
    Provides portable RGB565 rectangles, compact text, and font
    rendering.

ui_pages
    Formats STATUS, CRSF, IMU, CONTROL, and CAN snapshots on a ui_canvas.

drv_st7789
    Owns ST7789 initialization, orientation, RGB565 framebuffer storage,
    2x pixel scaling, controller commands, and display updates.

bsp_display_spi_esp32
    Owns the ESP32 SPI3 device, D/C pin, reset pin, and transfer
    chunking.

task_ui
    Owns the display instance and static framebuffer.
    Collects RC, IMU, control, safety, and actuator snapshots.
    Runs safety freshness checks.
    Feeds the portable input and UI state machines.
    Publishes timestamped manual-drive and torque-control requests.
    Does not grant motion permission or write CAN commands.
```

## Display orientation and memory use

Edit `APP_DISPLAY_ORIENTATION` in:

```text
04_firmware/platforms/esp32/main/config/config_ui.h
```

Available values are:

```c
DRV_ST7789_ORIENTATION_0
DRV_ST7789_ORIENTATION_90
DRV_ST7789_ORIENTATION_180
DRV_ST7789_ORIENTATION_270
```

The default is `DRV_ST7789_ORIENTATION_90`, which presents a landscape
160 x 120 logical canvas on the physical 320 x 240 orientation. The
portrait orientations automatically present a 120 x 160 canvas. Each
logical pixel is sent as a 2 x 2 physical block, keeping the static
framebuffer at 38,400 bytes. Page rendering reads the canvas dimensions
and does not depend on an ESP32 pin or ST7789 command.

## CAN manual-drive interaction

```text
1. Keep SD and SC low and center CH2.
2. Move SC high to enable UI input.
3. Browse to CAN with CH1.
4. Keep CH2 centered and move SD high.
5. When DRV LIVE appears, CH2 becomes the bounded M1 velocity input.
6. Move SD low, move SC low, or leave CAN to stop.
```

The UI request passes through `task_supervisor` and
`safety_manual_drive` before `task_motor` can transmit it. The display
never directly enables torque or writes a motor frame.

## CONTROL fixture-current interaction

```text
1. Keep SD and SC low and center CH2.
2. Move SC high to enable UI input.
3. Browse to CONTROL with CH1.
4. Use CH2 in Browse mode to choose state, snapshot, or torque view.
5. Center CH2 and move SD high.
6. Wait for TORQUE LIVE, then move CH2 gently to request current.
7. Move SD low, move SC low, or leave CONTROL to stop.
```

The UI request passes through `task_supervisor`, `task_control`, and
`safety_torque_control` before `task_motor` can transmit. A hard running fault
requires SD to be cycled before rearming.

## Reassigning controls

Edit:

```text
04_firmware/platforms/esp32/main/config/config_ui.h
```

Current assignments:

```c
#define APP_UI_RC_PAGE_AXIS_CHANNEL      1u
#define APP_UI_RC_VERTICAL_AXIS_CHANNEL  2u
#define APP_UI_RC_INTERACT_CHANNEL       6u
#define APP_UI_RC_ENTER_CHANNEL          7u
#define APP_UI_RC_INPUT_ENABLE_CHANNEL  10u
```

Manual-drive assignments are separately configurable in
`platforms/esp32/main/config/config_actuator.h`:

```c
#define APP_MANUAL_DRIVE_VELOCITY_CHANNEL 2u
```

The fixture current input is separately configurable in
`platforms/esp32/main/config/config_control.h`:

```c
#define APP_CONTROL_TORQUE_INPUT_CHANNEL 2u
```

Direction and polarity are also configurable:

```c
#define APP_UI_RC_PAGE_RIGHT_HIGH          1
#define APP_UI_RC_VERTICAL_UP_HIGH         1
#define APP_UI_RC_INTERACT_ACTIVE_HIGH     1
#define APP_UI_RC_ENTER_ACTIVE_HIGH        1
#define APP_UI_RC_INPUT_ENABLE_ACTIVE_HIGH 1
```

All five assigned channels must be unique. Invalid assignments or
threshold ordering prevent the UI task from starting.

SC uses separate thresholds so only its high position enables input.
Its low and middle positions lock input. The action-switch thresholds
remain suitable for the two-position SD switch and momentary SE switch.

## CRSF raw channel values

CRSF packed channels use 11-bit fields, but the normal control range is
narrower than the numeric `0-2047` field capacity.

```text
Raw 172     Extended low endpoint, approximately 988 us
Raw 191     Nominal low endpoint, 1000 us
Raw 992     Center endpoint, 1500 us
Raw 1792    Nominal high endpoint, 2000 us
Raw 1811    Extended high endpoint, approximately 2012 us
```

The receiver path accepts `172-1811` in `config_rc.h` as the protocol validity
range. The CRSF snapshot still carries all 16 protocol slots, but
`APP_RC_ACTIVE_CHANNEL_COUNT` is currently 10 so unused CH11-CH16 do
not block safety. UI neutral and activation thresholds are separate
values in `config_ui.h`.

## Future tuning boundary

Future tuning should introduce an explicit edit state and a
configuration service. The CAN DRIVE request is a narrowly scoped
commissioning action, not a general configuration mechanism.

```text
UI requests configuration mode.
Supervisor accepts only from a non-moving safe state.
Safety continues denying motion.
CH2 selects a setting.
S1 adjusts a pending value.
SE confirms an explicit action.
Control configuration validates bounds.
An explicit save action writes persistent storage.
```

The UI must never write controller globals, motor commands, torque
state, or persistent storage directly.
