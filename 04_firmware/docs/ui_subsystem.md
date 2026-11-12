# UI subsystem

## Operator model

The UI has two screens and no navigation hierarchy.

```text
READY
    shown when balance is not requested
    displays lean, lean rate, lean trim, RC/IMU/CAN health, and motor health
    SE captures the current stationary lean as zero

ACTIVE
    shown when SA and SD are on
    displays safety state/inhibit, lean/rate, commanded body velocity,
    estimated body velocity, and forward/lateral/yaw current contributions
```

## RC assignments

```text
SA / CH5    master enable
SD / CH6    balance enable
SE / CH7    zero lean request
Ele / CH3   forward/reverse velocity
Ail / CH1   lateral velocity
Rud / CH4   yaw rate
```

Assignments and switch polarities are in
`platforms/esp32/main/config/config_rc.h`.

SA and SD are read as levels. If a switch is on when the receiver becomes
usable, its state is applied immediately. The operator is responsible for
keeping both switches off before applying power.

SE is edge-triggered so holding the momentary switch does not repeatedly zero
lean. Zeroing is accepted only while the supervisor is in `SAFE_IDLE`, the
mounted IMU attitude is calibrated, valid, and fresh, raw lean is within the
configured zeroing window, and lean rate is small.

The ACTIVE screen uses `CMD` for the slew-limited reference, `VEL` for the
estimated motion, and `OUT` for the three controller current contributions.
The `OUT` row turns red when an axis or final motor allocation is saturated.
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
    renders READY or ACTIVE using read-only snapshots

task_ui
    owns the ST7789 display
    consumes snapshots only and may block during an LCD transfer

task_supervisor
    accepts only fresh requests
    selects SAFE_IDLE or BALANCE_CONTROL
```

The display is not part of the motion-request path. A slow or failed display
update cannot make the supervisor request stale.
