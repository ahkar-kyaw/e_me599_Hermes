# UI subsystem

## Operator model

The UI has no navigation hierarchy.

```text
READY
    shown when balance is not requested
    displays lean, lean rate, trim, subsystem health, and OTA Wi-Fi status

ACTIVE
    shown when SA and SD are on
    displays safety state/inhibit, lean/rate, commanded body velocity,
    estimated body velocity, and output saturation

UPDATE
    replaces the dashboard during OTA
    displays download progress and result
```

## RC assignments

```text
SA / CH5    master enable
SD / CH6    balance enable
SE / CH7    zero lean request
Ele / CH3   forward/reverse velocity
Ail / CH1   lateral velocity
Rud / CH4   yaw rate
SC / CH10   OTA trigger while disarmed
```

## Operator quick reference

1. Start with SA, SD, and SC off.
2. On READY, use SA on with SD off and press SE while the robot is stationary
   to capture the upright lean zero.
3. Turn SD on when the robot is upright and the motion sticks are centered.
4. In ACTIVE, Ele commands forward/reverse, Ail commands lateral motion, and
   Rud commands yaw.
5. Turn SA or SD off to stop.
6. For OTA, keep SA and SD off, start the Mac OTA server, then move SC low to
   high. The UPDATE page shows connection, discovery, and download state.

`TRIM` is the captured upright lean offset. Control subtracts it from the raw
mounted lean measurement so the current fixture/mechanical upright position is
reported as zero. It is updated by an accepted SE press and is not retained
after reboot.

Motion and safety assignments and switch polarities are in
`platforms/esp32/main/config/config_rc.h`. The OTA channel is selected with
`idf.py menuconfig`.

SA and SD are read as levels. If a switch is on when the receiver becomes
usable, its state is applied immediately. The operator is responsible for
keeping both switches off before applying power.

SE is edge-triggered so holding the momentary switch does not repeatedly zero
lean. Zeroing is accepted only while the supervisor is in `SAFE_IDLE`, the
mounted IMU attitude is calibrated, valid, and fresh, raw lean is within the
configured zeroing window, and lean rate is small.

The ACTIVE screen uses `CMD` for the slew-limited reference and `VEL` for the
estimated motion. `OUTPUT LIMITED` appears only during saturation.
`LOCKOUT / SD OFF` means a running fault was latched; lower SD or SA before
attempting another arm.

## Module ownership

```text
app_operator_controls
    maps three configurable CRSF channels to master, balance, and zero actions
    owns switch hysteresis and the SE rising edge

task_operator
    runs independently of display timing
    publishes the fresh timestamped app_balance_control_request_t

ui_dashboard
    renders READY, ACTIVE, or UPDATE using read-only snapshots

task_ui
    owns the ST7789 display
    consumes snapshots only and may block during an LCD transfer

task_supervisor
    accepts only fresh requests
    selects SAFE_IDLE, BALANCE_CONTROL, or UPDATE

task_ota
    keeps Wi-Fi off during normal operation
    accepts a low-to-high SC transition only while disarmed
    connects, discovers the Mac server, and publishes update progress
```

The display is not part of the motion-request path. A slow or failed display
update cannot make the supervisor request stale.
