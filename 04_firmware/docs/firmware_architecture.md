# Firmware architecture

## Layers

```text
common/
    Portable types, protocols, device drivers, estimation, control, safety,
    and UI drawing. No ESP-IDF, FreeRTOS, board pins, or MCU headers.

platforms/esp32/main/bsp/
    ESP32 peripheral backends.

platforms/esp32/main/services/
    Platform services that are not periodic RTOS behavior.

platforms/esp32/main/tasks/
    FreeRTOS scheduling, peripheral ownership, and subsystem integration.

platforms/esp32/main/config/
    Application-selected values and timing.

platforms/esp32/include/
    Board pin and bus mapping.

tests/
    Host tests for portable production logic.
```

Generated ESP-IDF files and local credentials remain outside portable code.
See [configuration.md](configuration.md) for authoritative configuration
locations.

## Ownership

| Module | Responsibility |
|---|---|
| `task_rc` | CRSF UART and latest receiver snapshot |
| `task_operator` | RC mapping and fresh operator intent |
| `task_imu` | IMU SPI, calibration, mounting, and attitude |
| `task_control` | State estimation and desired motor currents |
| `task_supervisor` | Whole-system mode |
| `task_safety` | Permission to move and approved currents |
| `task_motor` | CAN ownership, feedback, and approved transmission |
| `task_ui` | ST7789 and read-only dashboard rendering |
| `task_ota` | SC trigger, OTA state, and rollback confirmation |
| `svc_ota_esp32` | On-demand Wi-Fi, server discovery, and installation |

Supervisor owns mode. Safety owns motion permission. Control never transmits a
motor command directly. UI and OTA consume snapshots and cannot bypass safety.

## Command path

```text
RC -> task_operator -> task_supervisor -> task_control
                                      -> task_safety -> task_motor -> CAN
IMU ---------------------> task_control -----^
CAN feedback ------------> task_control -----^
```

Subsystem state crosses task boundaries as timestamped snapshots. Inputs used
by control or safety carry validity and freshness checks. A deliberate stop
sends zero current for the configured stop interval. A running fault also
latches `LOCKOUT`, requiring SA or SD to be lowered before rearming.

Display transfers cannot delay operator requests because `task_operator` is
independent of `task_ui`. Wi-Fi is inactive until an accepted SC low-to-high
OTA request places the supervisor in UPDATE.

## Naming

```text
task_    long-running RTOS task
board_   board mapping or wrapper
bsp_     MCU peripheral backend
drv_     device driver
proto_   wire protocol
svc_     service
app_     application glue and shared types
ctrl_    control
est_     estimation
safety_  safety logic
config_  application configuration file
ui_      user interface
```

Public functions use the owning module prefix. Macros use uppercase module
prefixes. Types use lowercase module prefixes with `_t`. Keep each module
focused on one responsibility and keep generated code separate from user-owned
code.
