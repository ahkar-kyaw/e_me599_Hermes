# State estimation and control

## Control state

The 200 Hz control task composes:

```text
position_x_m
position_y_m
yaw_rad
lean_rad
velocity_x_mps
velocity_y_mps
yaw_rate_rps
lean_rate_rps
timestamp_us
valid
```

Wheel velocity feedback feeds `est_collinear_mecanum`. Mount-corrected IMU
attitude supplies lean and lean rate. `APP_CONTROL_LEAN_AXIS` selects roll or
pitch explicitly.

## Lean trim

The control-facing lean is:

```text
lean = raw_mounted_lean - active_lean_trim
```

The trim starts at `APP_BALANCE_INITIAL_LEAN_TRIM_RAD`. Pressing SE on READY
captures raw lean when:

- the supervisor is in `SAFE_IDLE`;
- the mounted IMU attitude is calibrated, valid, and fresh;
- raw lean is inside `APP_BALANCE_ZERO_MAX_ABS_LEAN_RAD`; and
- lean rate is below `APP_BALANCE_ZERO_MAX_LEAN_RATE_RPS`.

This is an upright-reference operation, not a replacement for correct IMU axis
configuration. A sensor that reports roughly 90 degrees while upright should
be fixed in `config_imu.h`, not zeroed.

## Control law

```text
current = output_sign * (
      Klean * lean
    + Krate * lean_rate
    - Kpos  * position_error
    - Kvel  * velocity
    - Ki    * integral(position_error))
```

Forward, lateral, and yaw references are generated from CH3, CH1, and CH4 and
limited in both speed and acceleration. Position and yaw references are
captured again whenever motion is not safety-approved, preventing stored error
during an arm or stop. Integral action is bounded and disabled by default until
the proportional and damping loops are stable.

The controller produces desired current only. `safety_balance_control` decides
whether that current may reach the actuator task.
