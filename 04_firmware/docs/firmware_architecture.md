# Firmware architecture

## Ownership

```text
task_imu
    owns IMU SPI, boot gyro calibration, mounting correction, and attitude

task_rc
    owns CRSF UART and publishes the latest receiver snapshot

task_operator
    maps SA, SD, and SE and publishes fresh operator intent

task_ota
    owns OTA trigger policy and update progress

task_motor
    owns CAN, actuator feedback, freshness, and frame transmission

task_ui
    owns the display and renders read-only snapshots

task_supervisor
    owns whole-system mode: SAFE_IDLE, BALANCE_CONTROL, or UPDATE

task_control
    owns the 200 Hz state estimator and desired current calculation

task_safety
    owns permission to move and the only approved motor command snapshot
```

## Balance command path

```text
SA + SD
  -> task_operator request
  -> task_supervisor mode
  -> task_control desired currents
  -> safety_balance_control
  -> task_safety approved currents
  -> task_motor
  -> CubeMars servo-current CAN frames
```

Every boundary uses a timestamped snapshot. Control state is invalidated by
stale IMU data, stale actuator feedback, CAN degradation, estimator failure, or
control-loop timing failure.

A deliberate SA/SD shutdown sends zero current during the stop window and
returns to `READY`. A fault while running sends the same zero-current sequence
but enters `LOCKOUT`. The operator must turn SD or SA off before another arm;
this prevents torque from returning automatically after an intermittent fault.

## Lean zero path

```text
READY + SA on + SD off + SE press
  -> incremented zero_lean_request_count
  -> supervisor snapshot
  -> task_control validates stationary state
  -> active lean trim updated
  -> published lean becomes raw lean minus trim
```

The trim is runtime-only. A reboot returns to
`APP_BALANCE_INITIAL_LEAN_TRIM_RAD`.

## Platform boundary

Portable modules contain no ESP-IDF, FreeRTOS, MCU SDK, or pin dependencies.
ESP32 tasks and BSPs provide scheduling and peripheral access.

`task_operator` is intentionally separate from `task_ui`. Full-frame ST7789
transfers are blocking, but display latency cannot delay the operator request
or force the supervisor back to `SAFE_IDLE`.

OTA networking is ESP32-only. `task_ota` owns policy; `svc_ota_esp32` owns
on-demand Wi-Fi, local server discovery, and HTTP transport. Portable control
code has no networking dependency.
