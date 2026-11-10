# Firmware architecture

The firmware is organized around clear ownership boundaries. Each layer owns one type of responsibility and depends only on lower-level or peer interfaces that are stable.

## High-level layers

```text
Application startup
    app_main or app_main_create_tasks

Platform tasks
    task_imu
    task_rc
    task_ui
    task_motor
    task_supervisor
    task_safety
    task_control
    future task_log

Platform BSP
    SPI, CAN, UART, GPIO, timers, ADC, and board-specific hardware backends

Common app glue
    app_imu_types
    app_rc_types
    app_actuator_types
    app_manual_drive_types
    app_supervisor_types
    app_balance_state

Common subsystem logic
    drivers
    estimation
    safety
    control
    protocols
    services

Common interfaces
    if_spi
    if_display_io
    if_can
    future if_uart, if_time
```

## Current IMU pipeline

```text
ESP32 SPI pins
    |
    v
bsp_imu_spi_esp32
    |
    v
if_spi_t
    |
    v
drv_ism330dhcx
    |
    v
est_imu_mount
    |
    v
est_imu_calibration
    |
    v
est_attitude
    |
    v
imu_snapshot_t --> safety_imu_check --> safety_imu_status_t
                                                |
actuator_snapshot_t                             |
    |                                           |
    v                                           v
est_collinear_mecanum ------------> app_balance_state_compose
                                                |
                                                v
                                          balance_state_t
                                                |
                                                v
                                        control_snapshot_t
```

## Current RC pipeline

```text
RP2 ExpressLRS receiver
    |
    v
ESP32 UART2
    |
    v
bsp_crsf_uart_esp32
    |
    v
proto_crsf
    |
    v
rc_snapshot_t with all 16 channels
    |
    +--> safety_rc_check
    |
    +--> ui_rc_input
    |       |
    |       v
    |   semantic UI events
    |
    +--> task_supervisor and task_safety manual-drive input path
```

`task_rc` owns the UART and publishes the latest timestamped snapshot.
The protocol parser is portable and contains no ESP-IDF, FreeRTOS, pin,
or board dependencies.

## Current display pipeline

```text
ESP32 SPI3 and control pins
    |
    v
bsp_display_spi_esp32
    |
    v
if_display_io_t
    |
    v
drv_st7789
    |
    v
RGB565 framebuffer
    ^
    |
ui_canvas <--- ui_pages
    ^
    |
ui_state <--- ui_rc_input
    ^
    |
RC, IMU, and actuator snapshots
    |
    v
task_ui
```

`task_ui` owns the display. It may read subsystem snapshots and present
status, but it does not own safety state or motion commands. Portable UI
input mapping, browse/interact state, and page rendering remain in
`common/`.

## Current actuator and manual-drive pipeline

```text
Four CubeMars AK40-10 actuator slots
    |
    v
Adafruit TJA1051T/3 CAN Pal
    |
    v
ESP32 TWAI at 1 Mbit/s
    |
    v
bsp_can_esp32
    |
    v
if_can_frame_t
    |
    v
proto_cubemars_ak
    |
    v
task_motor
    |
    v
actuator_snapshot_t
    |
    +--> task_ui
    +--> task_safety

task_ui manual-drive request
    |
    v
task_supervisor mode
    |
    v
safety_manual_drive permission and bounded command
    |
    v
task_motor command freshness check and CAN transmit
```

`task_motor` publishes fresh servo feedback, fault codes, bus state, and
communication counters. It can transmit the bounded servo velocity
command only while a fresh `task_safety` approval is present. It sends
no command automatically at boot.

## Ownership rules

```text
task_imu
    Owns IMU hardware access.
    Publishes the latest IMU snapshot.

task_rc
    Owns CRSF UART access.
    Publishes the latest 16-channel RC snapshot.

task_ui
    Owns the ST7789 display and static RGB565 framebuffer.
    Reads snapshots, updates the portable UI state, and displays status.

task_motor
    Owns the ESP32 TWAI peripheral and CAN receive queue.
    Publishes feedback for four configurable actuator slots.
    Monitors bus state and requests recovery after bus-off.
    Sends only fresh, safety-approved commands.
    Applies a local zero-command fallback when approval expires.

task_supervisor
    Owns SAFE_IDLE, MANUAL_DRIVE, and TORQUE_CONTROL whole-system modes.
    Converts a fresh UI request into an explicit system mode.

task_safety
    Owns permission to move.
    Checks RC, state, IMU, CAN, feedback, motor faults, command bounds,
    neutral stick, and supervisor mode.
    Publishes the only command accepted by task_motor.

ui_rc_input
    Owns channel mapping, SC input gating, thresholds, hysteresis, and
    input re-arming.

ui_state
    Owns locked/browse/interact behavior, page, selection, and Enter
    events.

ui_pages
    Owns STATUS, CRSF, IMU, CONTROL, and CAN page formatting.
    Displays safety state but does not grant permission to move.

ui_canvas
    Owns portable RGB565 drawing and compact font rendering.

drv_ism330dhcx
    Owns ISM330DHCX register operations and raw-data conversion.

est_imu_mount
    Owns sensor-frame to body-frame mapping.

est_imu_calibration
    Owns gyro bias estimation and correction.

est_attitude
    Owns attitude estimation.

safety_imu
    Owns IMU-level validity and freshness checks.

safety_rc
    Owns RC validity, freshness, and raw channel-range checks.

est_collinear_mecanum
    Owns portable four-wheel planar odometry.

app_balance_state
    Owns composition of checked IMU lean state and wheel odometry.

ctrl_balance_types
    Owns the full control-facing state and timing diagnostic definitions.

task_control
    Runs state estimation and desired-command calculation at 200 Hz.
    Has no motor-transmission or safety-approval authority.
```

## Planned full robot task flow

```text
Supervisor task
    Owns whole-system mode.
    Coordinates safe idle, calibration, ready, armed, balancing, controlled stop, fault, and estop states.

Safety task
    Owns permission to move.
    Checks IMU, RC link, battery, motor feedback, command freshness, and system mode.

Control task
    Owns desired command calculation.
    Uses balance_state_t and user command inputs.
    Produces desired robot or motor commands.

Motor task
    Owns physical motor communication.
    Sends only safety-approved commands.

UI task
    Owns display and user interaction.
    Sends requests through queues or service interfaces.
    Does not directly enable torque or write motor commands.

Logging task
    Records snapshots, faults, command data, and experiment data.
```

## Data passing rules

```text
Use snapshots for latest state.
Use queues for events that must not be missed.
Use state machines for system mode, safety, UI, calibration, logging, and subsystem behavior.
Use timestamps and validity flags for data freshness.
```

## Build ownership

```text
common/
    Portable source files are compiled into each platform target as needed.

platforms/esp32/main/CMakeLists.txt
    Adds ESP32 files and selected common files to the ESP-IDF component.

platforms/stm32/app/CMakeLists.txt
    Adds user-owned STM32 app files and selected common files to the STM32 target.
```
