# Robot firmware

This firmware is organized to support an ESP32 bring-up platform and
an STM32H723ZG target platform while keeping reusable robot logic
portable. The active ESP32 target includes the IMU pipeline, a
16-channel CRSF receiver path, an ST7789 status display, and a
CubeMars feedback path with safety-gated manual velocity control.
The current milestone also provides safety-gated four-actuator current control
for restrained fixture commissioning.

## Current targets

| Target | Purpose | Toolchain |
| --- | --- | --- |
| ESP32 NodeMCU | Prototype bring-up and hardware validation | ESP-IDF, CMake, and Ninja |
| STM32H723ZG Nucleo | Final target platform | STM32CubeMX, CMake, Ninja, Arm GNU Toolchain |

## Repository layout

```text
04_firmware/
    common/
        include/
        src/

    platforms/
        esp32/
        stm32/

    docs/
```

## Layer responsibilities

```text
common/
    Portable robot logic.
    No ESP-IDF headers.
    No STM32 HAL headers.
    No FreeRTOS headers.
    No MCU pin numbers.
    No board-specific assumptions.

platforms/esp32/
    ESP32 startup, build files, board definitions, BSPs, and FreeRTOS tasks.

platforms/stm32/
    STM32CubeMX generated code, user-owned app code, and build files.

docs/
    Firmware architecture notes, naming rules, subsystem notes, and electrical interface documentation.
```

## Active ESP32 firmware flow

```text
app_main
    starts task_imu
    starts task_motor when APP_ENABLE_ACTUATORS is enabled
    starts task_control when APP_ENABLE_CONTROL_STATE is enabled
    starts task_rc and task_ui when enabled
    starts task_supervisor and task_safety for manual commissioning

task_imu
    owns the IMU hardware
    initializes ESP32 SPI through the BSP
    initializes the ISM330DHCX driver
    applies IMU mounting orientation
    performs gyro bias calibration
    runs the attitude estimator
    publishes imu_snapshot_t

task_rc
    owns UART2 and receives RadioMaster RP2 CRSF data
    publishes all 16 raw channels with timestamps and error counters

task_ui
    owns the ST7789 SPI display
    shows STATUS, CRSF, IMU, and CAN pages
    uses a portable locked/browse/interact UI state machine
    publishes a timestamped CAN-page manual-drive request

task_supervisor
    owns SAFE_IDLE, MANUAL_DRIVE, and TORQUE_CONTROL whole-system modes

task_safety
    validates RC, supervisor mode, state, IMU, CAN, feedback, and limits
    publishes the only actuator command accepted by task_motor

task_motor
    owns ESP32 TWAI through bsp_can_esp32
    monitors four configurable CubeMars actuator slots
    decodes timestamped servo feedback and driver faults
    transmits only fresh, safety-approved servo velocity or current commands
    sends no command automatically at boot

task_control
    runs the portable state estimator at 200 Hz
    combines four-wheel odometry with checked IMU lean state
    publishes timing, validity, state, and desired-current diagnostics
    maps CH2 to bounded fixture current in TORQUE_CONTROL mode
    never transmits motor commands directly

```

## Common firmware flow

```text
if_spi
    generic SPI transfer interface

if_display_io
    generic command/data/reset display interface

if_can
    generic classic CAN frame representation

proto_crsf
    portable CRSF stream parser and 16-channel unpacker

proto_cubemars_ak
    portable CubeMars servo command packing and feedback decoding

drv_st7789
    portable ST7789 RGB565 driver with configurable orientation

safety_rc
    checks RC validity, freshness, and raw channel range

ui_rc_input
    maps reassignable CRSF channels to semantic UI events

ui_state
    owns locked/browse/interact behavior, page, and selection

ui_pages
    renders portable status and manual-drive state through ui_canvas

safety_manual_drive
    owns the portable manual commissioning gate and arm state machine

safety_torque_control
    owns portable fixture-current safety gates, stop, and rearm state

ctrl_torque_input
    maps a raw RC channel to bounded, slew-limited desired current

drv_ism330dhcx
    portable ISM330DHCX register driver

est_imu_mount
    maps sensor-frame measurements into robot body frame

est_imu_calibration
    estimates and removes gyro bias

est_attitude
    generic attitude estimator API
    currently uses a complementary filter
    prepared for Madgwick or Kalman later

safety_imu
    checks IMU validity, calibration, freshness, tilt, and acceleration magnitude

est_collinear_mecanum
    converts four wheel speeds into planar position, velocity, and yaw

app_balance_state
    combines checked IMU lean state with wheel odometry

ctrl_balance_types
    defines the full eight-state control snapshot and timing diagnostics
```

## ESP32 build commands

Activate the ESP-IDF environment, then run these from the repository
root.

```bash
. ~/esp/esp-idf-v5.5/export.sh
cd 04_firmware/platforms/esp32
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

Run `idf.py set-target esp32` once after cloning the repository or when
resetting the generated project configuration. Use `idf.py menuconfig`
to inspect or change ESP-IDF configuration.

## STM32 build commands

Run these from the STM32 platform folder.

```bash
cd 04_firmware/platforms/stm32
cmake --preset Debug
cmake --build --preset Debug
```

## Build-time flags

```text
APP_ENABLE_RC_RECEIVER
    Starts the ESP32 CRSF receiver task.

APP_ENABLE_UI
    Starts the ESP32 UI task.

APP_ENABLE_ACTUATORS
    Starts the ESP32 actuator task. It always receives feedback and
    accepts only fresh commands approved by task_safety.

APP_ENABLE_CONTROL_STATE
    Starts the 200 Hz state-estimation and desired-command task. Motor
    transmission still requires supervisor and safety approval.

```

## Development rules

```text
Keep common code portable.
Keep platform code isolated.
Keep temporary tests under tests/.
Keep long-term tasks under tasks/.
Keep board pin maps in board headers.
Keep MCU peripheral setup in BSP modules.
Keep safety checks separate from control calculations.
Do not let UI, tests, or control write directly to motors.
```

## Documentation map

```text
04_firmware/README.md
    Firmware entry point and build guide.

04_firmware/common/README.md
    Portable common-layer organization.

04_firmware/platforms/esp32/README.md
    ESP32 bring-up platform guide.

04_firmware/platforms/stm32/README.md
    STM32 target platform guide.

04_firmware/docs/firmware_architecture.md
    Firmware architecture and data flow.

04_firmware/docs/software_guide_and_roadmap.md
    Comprehensive current-software guide, modification map, limitations,
    and staged roadmap to the final robot.

04_firmware/docs/naming_conventions.md
    File, module, task, and macro naming rules.

04_firmware/docs/imu_subsystem.md
    IMU driver, calibration, mounting, estimator, safety, and balance-state pipeline.

04_firmware/docs/state_estimation_and_control.md
    Four-wheel odometry, state composition, timing, configuration, and tests.

04_firmware/docs/ui_subsystem.md
    LCD pages, RC controls, UI state machine, CRSF ranges, and future tuning boundary.

04_firmware/docs/actuator_subsystem.md
    CubeMars servo CAN, four-motor configuration, feedback snapshots,
    safety ownership, bring-up, and MIT-mode growth path.

04_firmware/docs/robot_configuration_checklist.md
    Values to verify from the assembled robot before fixture and balance tests.

04_firmware/docs/electrical_interfaces.md
    Firmware-facing electrical interface notes and pin maps.
```
