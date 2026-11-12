# Firmware

ESP-IDF firmware for a four-actuator self-balancing mobile robot. Portable
logic lives in `common`; ESP32 peripheral ownership and FreeRTOS integration
live in `platforms/esp32`.

## Runtime architecture

```mermaid
flowchart LR
    RC[CRSF receiver] --> TR[task_rc]
    TR --> TO[task_operator]
    TO --> TS[task_supervisor]
    TO --> OTA[task_ota]

    IMU[ISM330DHCX] --> TI[task_imu]
    CAN[CubeMars CAN] <--> TM[task_motor]
    TI --> TC[task_control<br/>estimate + balance + motion + allocation]
    TM --> TC
    TS --> TC

    TC -->|desired current| SAFE[task_safety]
    TS --> SAFE
    TI --> SAFE
    TR --> SAFE
    TM --> SAFE
    SAFE -->|approved current only| TM
    MAC[Mac OTA server] <-->|Phone hotspot LAN| OTA
    OTA --> FLASH[Inactive OTA slot]
    OTA --> TS

    TO -. snapshots .-> UI[task_ui]
    TI -. snapshots .-> UI
    TM -. snapshots .-> UI
    TC -. snapshots .-> UI
    SAFE -. snapshots .-> UI
    UI --> LCD[ST7789 dashboard]
```

The dashed UI paths are read-only. The display cannot enable torque or write
motor commands. `task_safety` is the only owner of motion permission.

## Operating modes

```mermaid
stateDiagram-v2
    [*] --> READY: Boot / SA off or SD off
    READY --> WAIT_SAFE: SA + SD on, sticks centered
    WAIT_SAFE --> LIVE: Inputs safe for 300 ms
    LIVE --> STOPPING: SA off, SD off, or inhibit
    STOPPING --> READY: Deliberate switch off
    STOPPING --> LOCKOUT: Running fault
    LOCKOUT --> READY: SD or SA off
    WAIT_SAFE --> READY: SA off or SD off
    READY --> UPDATE: SA + SD off, SC low then high
    UPDATE --> READY: Failed update, SC low
    UPDATE --> [*]: Successful update reboots
```

| Mode | Expected behavior | How to enter |
|---|---|---|
| `READY` | Motors receive no drive current. The display shows lean, trim, RC, IMU, CAN, and motor health. | Keep SA or SD off. With SA on and SD off, press SE while stationary to zero lean. |
| `WAIT SAFE` | Current remains disabled until the robot and centered motion inputs remain inside all arming limits for 300 ms. | Turn SA on, then SD on. Hold the robot near upright and still with CH1, CH3, and CH4 centered. |
| `LIVE` | Closed-loop balance, forward, lateral, yaw, and bounded position recovery are active. | Complete `WAIT SAFE` with no inhibit. |
| `STOPPING` | Zero-current CAN commands are sent for 200 ms before transmission stops. | Turn SA or SD off, lose a required input, exceed the fall limit, or encounter a motor/CAN fault. |
| `LOCKOUT` | Torque cannot automatically return after a running fault. | Turn SD or SA off to acknowledge the stop, correct the cause, then arm again. |
| `UPDATE` | Wi-Fi connects, the Mac is discovered, and motion remains blocked while the image is installed. | With SA and SD off, move SC low and high. |

## Source layout

```text
common/
    include/ and src/    portable drivers, protocols, estimation, control,
                        safety, UI drawing, and shared types

platforms/esp32/
    main/bsp/            ESP-IDF peripheral backends
    main/config/         pin-independent application configuration
    main/tasks/          FreeRTOS task ownership and subsystem glue

tests/                   host tests for portable logic
```

## Operator controls

```text
SA / CH5    master enable
SD / CH6    balance request
SE / CH7    zero lean in READY while stationary
Ele / CH3   forward and reverse velocity in LIVE
Ail / CH1   lateral velocity in LIVE
Rud / CH4   yaw rate in LIVE
SC / CH10   OTA trigger while SA and SD are off
```

Motion and safety channels are configured in
`platforms/esp32/main/config/config_rc.h`. The OTA channel is configured with
`idf.py menuconfig`.

## Build and flash

```bash
cd 04_firmware/platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

Host tests:

```bash
cd 04_firmware/tests
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Documentation is indexed in [`docs/README.md`](docs/README.md).
