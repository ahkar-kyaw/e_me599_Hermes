# ESP32 platform firmware

The ESP32 platform is the active bring-up target. It uses native
ESP-IDF with CMake and Ninja and runs the IMU pipeline, CRSF receiver,
ST7789 READY/ACTIVE dashboard, CubeMars feedback, state estimation, and
safety-gated balance-current control.

## Structure

```text
platforms/esp32/
    CMakeLists.txt
    sdkconfig.defaults

    include/
        board_esp32_nodemcu_v1.h

    main/
        CMakeLists.txt
        main.c

        bsp/
            bsp_can_esp32.c
            bsp_can_esp32.h
            bsp_crsf_uart_esp32.c
            bsp_crsf_uart_esp32.h
            bsp_display_spi_esp32.c
            bsp_display_spi_esp32.h
            bsp_imu_spi_esp32.c
            bsp_imu_spi_esp32.h

        config/
            config_actuator.h
            config_imu.h
            config_rc.h
            config_ui.h

        tasks/
            task_imu.c
            task_imu.h
            task_motor.c
            task_motor.h
            task_operator.c
            task_operator.h
            task_rc.c
            task_rc.h
            task_safety.c
            task_safety.h
            task_supervisor.c
            task_supervisor.h
            task_ui.c
            task_ui.h

```

## Toolchain

The current firmware is validated with Espressif's ESP-IDF `v5.5`
release tag. Activate the ESP-IDF environment before running any
`idf.py` command:

```bash
. ~/esp/esp-idf-v5.5/export.sh
```

The generated `sdkconfig` and `build/` directory are local build
artifacts. Project-owned defaults belong in `sdkconfig.defaults`.

## Configure, build, and upload

Run from the repository root.

```bash
cd 04_firmware/platforms/esp32
idf.py set-target esp32
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

`idf.py set-target esp32` is required after cloning or when resetting
the generated configuration. It does not need to run before every
build. Use `idf.py menuconfig` for interactive ESP-IDF configuration.
Exit the serial monitor with `Ctrl+]`.

## Build-time flags

```text
APP_PLATFORM_ESP32
BOARD_ESP32_NODEMCU_V1
APP_ENABLE_ACTUATORS
APP_ENABLE_RC_RECEIVER
APP_ENABLE_UI
```

`APP_ENABLE_ACTUATORS` starts the motor feedback and safety-approved
command task. No command is transmitted automatically at boot.
`APP_ENABLE_RC_RECEIVER` starts the receiver task.
`APP_ENABLE_UI` starts the display task.
The current values are defined in `main/CMakeLists.txt`.

## Board pin map

```text
IMU SCLK        GPIO18
IMU MISO        GPIO19
IMU MOSI        GPIO23
IMU CS          GPIO27
IMU INT1        GPIO34
IMU INT2        GPIO35

Display CLK     GPIO14
Display DIN     GPIO13
Display CS      GPIO25
Display D/C     GPIO21
Display RST     GPIO22

CRSF UART RX    GPIO16, connected to RP2 TX
CRSF UART TX    GPIO17, connected to RP2 RX

CAN TX          GPIO26, connected to CAN Pal TX
CAN RX          GPIO32, connected to CAN Pal RX
```

INT1 and INT2 are defined for future use. The IMU is currently polled.
The full electrical pin map and power notes are in
`03_electrical/docs/pin_map_esp32_bringup.md`.

## BSP layer

```text
bsp_imu_spi_esp32
    Initializes the ESP32 SPI bus.
    Implements if_spi for drv_ism330dhcx.

bsp_display_spi_esp32
    Initializes the dedicated ESP32 SPI3 display bus at 20 MHz.
    Owns display D/C and reset GPIO.
    Implements if_display_io for drv_st7789.

bsp_crsf_uart_esp32
    Initializes UART2 for non-inverted 420000-baud CRSF.
    Provides buffered UART reads to task_rc.

bsp_can_esp32
    Initializes the ESP32 TWAI controller at 1 Mbit/s.
    Receives classic CAN frames through an ISR-to-task queue.
    Reports bus state and supports bus-off recovery.
```

## Task layer

```text
task_imu
    Owns the IMU device and publishes imu_snapshot_t.

task_rc
    Owns the receiver UART.
    Feeds bytes to the portable CRSF parser.
    Publishes all 16 channels in rc_snapshot_t with timestamp and
    communication error counters.

task_operator
    Maps SA, SD, and SE through the portable operator-control module.
    Runs every 20 ms independently of the display.
    Publishes the timestamped balance-control request consumed by the
    supervisor.

task_ui
    Owns the ST7789 and static 160 x 120 / 120 x 160 RGB565 canvas.
    Collects RC, IMU, control, safety, and actuator snapshots.
    Renders the READY or ACTIVE dashboard.
    Is read-only and does not own requests, safety state, or motion commands.

task_supervisor
    Owns SAFE_IDLE and BALANCE_CONTROL modes.

task_safety
    Owns permission to move.
    Checks RC, state, IMU, CAN, feedback, faults, command bounds,
    and supervisor gates.
    Publishes the only approved command accepted by task_motor.

task_motor
    Owns the TWAI BSP.
    Decodes CubeMars periodic servo feedback.
    Publishes timestamped feedback for four configurable actuator slots.
    Transmits only fresh, safety-approved current commands.
    Sends no command automatically at boot.
```

## Configuration

```text
config_imu.h
    IMU sampling, mounting, estimation, calibration, and safety values.

config_rc.h
    RC freshness, active channels, raw range, operator switch mapping,
    operator task period, UART timeout, and retry.

config_ui.h
    ST7789 mounting orientation and UI timing.

config_actuator.h
    CAN bitrate, freshness, and four actuator enable/ID entries.
    Command freshness and zero-current hold time.
    Motors 1 through 4 are enabled at decimal CAN IDs 1 through 4.
    Per-slot direction signs convert physical mounting to robot signs.

config_control.h
    State geometry and conversion values.
    Balance gains, lean zero limits, current limit, motor mix,
    freshness, stop hold, and lean safety limits.
```

Current UI mapping:

```text
CH5 SA     master enable
CH6 SD     balance off/on
CH7 SE     zero stationary lean on READY
```

See `04_firmware/docs/ui_subsystem.md` for UI behavior, channel
reassignment, CRSF raw ranges, and the future tuning boundary.
See `04_firmware/docs/actuator_subsystem.md` for CubeMars protocol,
wiring, configuration, safety ownership, and the MIT-mode growth path.

Body frame convention:

```text
Body X positive    robot front
Body Y positive    robot left
Body Z positive    robot up
```

## ESP32 design rules

```text
Keep pin definitions in include/board_*.h.
Keep ESP-IDF peripheral setup in bsp/.
Keep long-running FreeRTOS tasks in tasks/.
Do not include ESP-IDF or FreeRTOS headers in common/.
Keep temporary bring-up code out of the final application.
```
