# State estimation and torque-control foundation

## Current scope

The active implementation runs state estimation and a bounded manual-current
commissioning path at 200 Hz. It is intended for restrained fixture testing.
It is not yet the closed-loop balance controller.

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
    -> desired current snapshot
    -> safety_torque_control
    -> safety-approved current snapshot
    -> task_motor
```

The UI only requests torque-control mode. The supervisor owns the system mode,
the control task computes desired current, the safety task decides whether it
may move, and the motor task alone owns CAN transmission.

## Coordinate and state convention

```text
+X    forward
+Y    left
+Z    up
yaw  positive counterclockwise about +Z
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

The wheel axle lies along body Y, so the configured balance lean is pitch about
body Y. `APP_CONTROL_LEAN_AXIS` keeps this mapping explicit.

## Wheel kinematics

The portable estimator uses:

```text
wheel_linear_speed[i] =
    body_forward_speed
    + roller_lateral_factor[i] * body_left_speed
    - wheel_lateral_position[i] * yaw_rate
```

It computes the least-squares inverse during initialization and uses the
precomputed matrix at runtime. Body velocity is rotated into the world frame
before position integration.

Robot geometry is configured in:

```text
04_firmware/platforms/esp32/main/config/config_control.h
```

The checked-in dimensions are placeholders until the assembled robot is
measured. Do not tune the balance controller from these values.

## Actuator conversion

CubeMars feedback reports electrical RPM:

```text
wheel rad/s = ERPM * 2*pi / (60 * pole_pairs * gear_ratio)
```

The current configuration uses seven pole pairs and a 10:1 reduction. Confirm
the mechanical ratio and wheel speed on the assembled robot. Protocol code
continues to expose ERPM and does not contain robot geometry.

## Fixture current control

CH2 maps to a signed current command with a neutral band, current limit, and
slew-rate limit. The configured mix applies that robot-level command to all
four actuator slots. The motor task applies each slot's direction sign before
packing the CubeMars servo-current CAN frame.

Current is commanded in amperes. It is not treated as verified output torque
until motor torque constant, gear efficiency, and drivetrain losses have been
measured.

Default commissioning limits:

```text
maximum current       1.0 A
current slew rate     4.0 A/s
control period        5 ms
arm lean limit        10 deg
arm lean-rate limit   80 deg/s
running fall limit    30 deg
```

These values live in `config_control.h`. Keep the initial current limit low and
increase it only from measured test results.

## Torque safety state machine

```text
DISABLED
    No torque-control request.

WAIT_SAFE
    A hard condition is invalid: RC, state, IMU, CAN, feedback, motor fault,
    or current bounds.

WAIT_NEUTRAL
    Conditions are healthy, but CH2 must return to neutral or SD must be
    cycled after a running fault.

ARMED
    Fresh bounded current commands are approved.

STOPPING
    Zero current is transmitted for a bounded stop window.
```

No actuator command is sent automatically at boot. Torque control requires the
CONTROL page, SC input enable, SD Interact request, healthy snapshots from all
four actuators, a safe IMU state, and centered CH2 before arming.

## Timing and validity

`task_control` runs every 5 ms. The snapshot records loop count, invalid-input
count, overrun count, execution time, maximum execution time, and input fault
flags.

State is invalid if IMU data is unavailable, uncalibrated, or stale; any
actuator feedback is missing, stale, or faulted; CAN is not error-active; the
update interval is invalid; or the estimator fails. Safety never approves an
invalid or stale desired command.

## Verification

Portable tests cover wheel kinematics, current mapping and slew limiting, and
the torque safety arm, fault-stop, and rearm sequence:

```bash
cmake -S 04_firmware/tests -B /tmp/app_common_tests
cmake --build /tmp/app_common_tests
ctest --test-dir /tmp/app_common_tests --output-on-failure
```

The next control milestone is a measured, model-based closed-loop balance law.
It should replace the fixture current source while preserving the existing
supervisor, safety, command snapshot, and motor ownership boundaries.
