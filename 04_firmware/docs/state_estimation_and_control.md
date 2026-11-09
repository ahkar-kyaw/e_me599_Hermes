# State estimation and deterministic control foundation

## Current scope

The active implementation creates the full state interface needed by future
balance control without transmitting control commands. It combines checked IMU
attitude with four-wheel odometry in a fixed-rate ESP32 task.

```text
CubeMars feedback
    signed electrical RPM
    -> configured motor and gearbox conversion
    -> wheel rad/s
    -> portable collinear-mecanum odometry

ISM330DHCX snapshot
    -> IMU safety and freshness checks
    -> configured lean axis and lean rate

odometry + IMU
    -> balance_state_t
    -> task_control at 200 Hz
    -> read-only control_snapshot_t
```

`task_control` is intentionally dry-run only. It has no motor-command API and
cannot call the motor transmit path.

## Coordinate and state convention

The firmware convention is:

```text
+X    forward
+Y    left
+Z    up
+yaw  positive counterclockwise about +Z
```

The eight-state order is:

```text
position_x_m
position_y_m
yaw_rad
lean_rad
velocity_x_mps
velocity_y_mps
yaw_rate_rps
lean_rate_rps
```

The wheel axle lies along body Y, so the default balance lean is pitch about
body Y. `APP_CONTROL_LEAN_AXIS` keeps this mapping explicit.

## Wheel kinematics

The portable estimator uses the relationship:

```text
wheel_linear_speed[i] =
    body_forward_speed
    + roller_lateral_factor[i] * body_left_speed
    - wheel_lateral_position[i] * yaw_rate
```

It computes the least-squares inverse once during initialization, then uses the
precomputed four-wheel-to-body matrix at runtime. Body velocity is rotated into
the world frame before position integration.

Geometry is configured in `config_control.h`:

```text
wheel radius              0.060 m
wheel lateral positions   +0.240, +0.080, -0.080, -0.240 m
roller lateral factors    +1, -1, +1, -1
```

These values follow the MATLAB model after mapping its right/forward axes into
the firmware forward/left convention.

## Actuator conversion

CubeMars feedback reports electrical RPM. The current conversion is explicit:

```text
motor poles       14
pole pairs         7
gear reduction    10:1
wheel rad/s = ERPM * 2*pi / (60 * 7 * 10)
```

The pole count comes from the bundled AK40-10 parameter file. Confirm the
physical output-speed ratio during commissioning before controller tuning.
Protocol code continues to expose ERPM and does not contain robot geometry.

## Timing

`task_control` runs every 5 ms. ESP-IDF is configured for a 1000 Hz FreeRTOS
tick so the period is representable. The snapshot records:

```text
loop count
invalid-input count
overrun count
last execution time
maximum execution time
input fault flags
```

A state is invalid if IMU data is unavailable, uncalibrated, or stale, any actuator feedback is missing,
stale, or faulted, CAN is not error-active, the update interval is invalid, or
the estimator fails. Invalid input is never silently reused as a valid control
state.

## Verification

Portable host tests cover pure forward, pure lateral, and pure yaw wheel-speed
patterns. Run them from the firmware directory:

```bash
cmake -S tests -B /tmp/app_common_tests
cmake --build /tmp/app_common_tests
ctest --test-dir /tmp/app_common_tests --output-on-failure
```

The ESP32 build includes the estimator and dry-run task. No balance gains or
motor commands are part of this milestone.
