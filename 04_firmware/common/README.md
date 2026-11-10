# Common firmware layer

The common layer contains portable firmware modules that can be reused by ESP32, STM32, and future platforms.

## Purpose

```text
common/
    Owns reusable robot logic.
    Does not own MCU startup.
    Does not own pin mapping.
    Does not own RTOS task creation.
    Does not include vendor HAL headers.
```

## Current structure

```text
common/
    include/
        app/
            app_actuator_types.h
            app_balance_state.h
            app_imu_types.h
            app_manual_drive_types.h
            app_rc_types.h
            app_supervisor_types.h

        control/
            ctrl_balance_types.h

        drivers/
            drv_ism330dhcx.h
            drv_st7789.h

        estimation/
            est_attitude.h
            est_collinear_mecanum.h
            est_imu_calibration.h
            est_imu_mount.h

        interfaces/
            if_can.h
            if_display_io.h
            if_spi.h

        protocols/
            proto_cubemars_ak.h
            proto_crsf.h

        safety/
            safety_imu.h
            safety_manual_drive.h
            safety_rc.h

        ui/
            ui_canvas.h
            ui_pages.h
            ui_rc_input.h
            ui_state.h
            ui_types.h

    src/
        app/
            app_balance_state.c

        drivers/
            drv_ism330dhcx.c
            drv_st7789.c

        estimation/
            est_attitude.c
            est_collinear_mecanum.c
            est_imu_calibration.c
            est_imu_mount.c

        protocols/
            proto_cubemars_ak.c
            proto_crsf.c

        safety/
            safety_imu.c
            safety_manual_drive.c
            safety_rc.c

        ui/
            ui_canvas.c
            ui_pages.c
            ui_rc_input.c
            ui_state.c
```

## Module responsibilities

### interfaces

```text
if_spi
    Defines a generic SPI transfer function.
    Allows common drivers to use SPI without knowing the MCU backend.

if_display_io
    Defines portable command, data, reset, and delay operations for a
    display controller.
    Hides MCU SPI and GPIO details from display drivers.

if_can
    Defines a portable classic CAN frame.
    Hides ESP32 TWAI and future STM32 FDCAN frame types from protocols.
```

### drivers

```text
drv_ism330dhcx
    Portable ISM330DHCX register driver.
    Probes WHO_AM_I.
    Configures ODR and full-scale settings.
    Reads raw temperature, gyro, and accelerometer data.
    Converts raw counts to SI units using the active driver configuration.

drv_st7789
    Portable ST7789 RGB565 display driver.
    Owns controller initialization, orientation, scaled pixel storage,
    rectangles, and display transfers through if_display_io.
```

### protocols

```text
proto_crsf
    Parses a CRSF byte stream.
    Validates DVB-S2 CRC-8.
    Unpacks all 16 11-bit RC channels.
    Tracks valid-frame, CRC-error, and parse-error counts.

proto_cubemars_ak
    Packs selected CubeMars servo commands into extended CAN frames.
    Decodes periodic servo feedback and fault codes.
    Leaves CAN peripheral ownership to the platform motor task.
```

### estimation

```text
est_imu_mount
    Maps sensor-frame acceleration and gyro data into the robot body frame.
    Body X is robot front.
    Body Y is robot left.
    Body Z is robot up.

est_imu_calibration
    Collects stationary gyro samples.
    Computes gyro bias.
    Applies bias correction to gyro data.

est_attitude
    Provides a generic attitude-estimator API.
    Currently implements a complementary filter.
    Reserves algorithm modes for Madgwick and Kalman filters.
    Supports 6DOF inputs now and 9DOF inputs later.

est_collinear_mecanum
    Precomputes a four-wheel least-squares kinematic inverse.
    Estimates planar position, velocity, yaw, and yaw rate in SI units.
    Contains no motor-protocol, RTOS, or platform dependencies.
```

### safety

```text
safety_imu
    Checks whether the IMU data is safe for balance use.
    Checks validity, calibration, attitude validity, timestamp freshness, tilt, and acceleration magnitude.

safety_rc
    Checks RC snapshot validity, freshness, and raw channel range.
    Does not grant whole-system permission to move.

safety_manual_drive
    Checks the supervisor request, RC, selected actuator, CAN state,
    feedback freshness, motor fault, and neutral-stick entry.
    Publishes a bounded manual velocity command and stop state.
```

### control

```text
ctrl_balance_types
    Defines the full eight-state balance snapshot and task diagnostics.
    Does not depend on IMU, actuator, RTOS, or safety modules.
```

### ui

```text
ui_rc_input
    Converts configured RC channels into page, vertical, interaction,
    Enter, input-gate, and link-loss events.

ui_state
    Portable locked/browse/interact state machine.
    Owns the current page and selection.

ui_canvas
    Portable RGB565 drawing surface and compact font renderer.

ui_pages
    Renders STATUS, CRSF, IMU, CONTROL, and CAN pages through ui_canvas.
    Displays commissioning safety state without granting permission.
```

### app

```text
app_imu_types
    Defines imu_snapshot_t.
    This is the shared IMU state published by platform tasks.

app_actuator_types
    Defines the four-slot actuator feedback snapshot, freshness state,
    driver faults, CAN state, and communication counters.

app_manual_drive_types
    Defines the UI request and safety-approved manual-drive snapshot.

app_torque_control_types
    Defines fixture-current requests, safety states, inhibits, and snapshots.

app_actuator_command_types
    Defines the generic safety-approved per-actuator command boundary.

app_supervisor_types
    Defines SAFE_IDLE, MANUAL_DRIVE, and TORQUE_CONTROL modes.

app_rc_types
    Defines rc_snapshot_t with all 16 raw CRSF channels, timestamp,
    validity, and communication error counters.

app_balance_state
    Combines checked IMU lean state and portable wheel odometry into
    balance_state_t. This keeps control types independent from source snapshots.
```

## Dependency direction

```text
interfaces
    used by drivers

drivers
    used by platform tasks and estimation modules

estimation
    used by platform tasks

safety
    used by platform safety tasks

control
    used by app glue and future control tasks

app
    allowed to connect subsystem types together

ui
    reads app and safety snapshots and uses the portable display driver
```

## Common-layer rules

```text
Allowed:
    stdint.h
    stdbool.h
    stddef.h
    string.h
    math.h
    common project headers

Not allowed:
    ESP-IDF headers
    STM32 HAL headers
    CubeMX generated headers
    FreeRTOS headers
    GPIO pin numbers
    MCU register access
    board names
```
