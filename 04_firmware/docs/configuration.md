# Firmware configuration

Configuration values have one authoritative source. Documentation names that
source instead of copying values that can become stale.

| Area | Authoritative source |
|---|---|
| Board pins and buses | `platforms/esp32/include/board_esp32_nodemcu_v1.h` |
| Actuator IDs, directions, and CAN timing | `platforms/esp32/main/config/config_actuator.h` |
| IMU mounting, sampling, and limits | `platforms/esp32/main/config/config_imu.h` |
| Geometry, motion, balance, and safety limits | `platforms/esp32/main/config/config_control.h` |
| RC channels, polarity, endpoints, and deadband | `platforms/esp32/main/config/config_rc.h` |
| Display orientation and timing | `platforms/esp32/main/config/config_ui.h` |
| OTA ports, path, and timeouts | `platforms/esp32/main/config/config_ota.h` |
| OTA enable, credentials, and trigger channel | ESP-IDF `sdkconfig` through `Kconfig.projbuild` |

## ESP-IDF configuration

Run:

```bash
cd 04_firmware/platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py menuconfig
```

`sdkconfig` is local and ignored by Git. ESP-IDF generates `sdkconfig.h`; OTA
code consumes `CONFIG_APP_OTA_ENABLE`, `CONFIG_APP_OTA_WIFI_SSID`,
`CONFIG_APP_OTA_WIFI_PASSWORD`, and `CONFIG_APP_OTA_TRIGGER_CHANNEL` through
`config_ota.h`. Do not copy credential values into source or documentation.

`sdkconfig.defaults` is the versioned, non-secret baseline for partitioning,
logging, networking features, and flash size.

## Robot-specific settings

Confirm these groups against the assembled robot before tuning:

- IMU forward/up axes, output data rates, full-scale ranges, freshness, and
  acceleration limits in `config_imu.h`.
- CAN bitrate, actuator IDs, physical direction signs, feedback freshness, and
  command timing in `config_actuator.h`.
- Wheel radius, wheel offsets, roller angles, gear ratio, pole pairs, body-axis
  signs, control gains, current limits, and arm/fall limits in
  `config_control.h`.
- RC channel assignments, endpoint range, switch thresholds, polarity, motion
  inversion, and deadband in `config_rc.h`.
- LCD mounting orientation in `config_ui.h`.

Change per-actuator direction only to correct an individual motor installation.
Change a body-axis output sign only when the complete forward, lateral, or yaw
response has the wrong sign.

## Model-based tuning inputs

Update the MATLAB model with measured mass, center-of-mass location, body and
wheel inertia, current-to-torque relationship, drivetrain efficiency,
friction, backlash, actuator response, IMU delay/noise, CAN latency, battery
limits, and safe thermal current. Do not transfer simulated gains until those
parameters and units match the assembled robot.
