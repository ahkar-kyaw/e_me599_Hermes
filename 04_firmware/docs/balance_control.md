# Balance and disturbance-rejection control

## Implemented scope

The portable controller stabilizes the forward balance plane using lean, lean
rate, forward position, forward velocity, and an optional bounded position
integral. It produces a common current distributed through four configurable
mix values.

Lateral position and yaw control are not implemented yet.

## Operator sequence

```text
SA off + SD off   SAFE_IDLE and READY screen
SA on + SD off    READY; SE may zero stationary lean
SA on + SD on     request BALANCE_CONTROL and show ACTIVE screen
SD off or SA off  zero-current stop and SAFE_IDLE
```

Switch levels apply directly. A switch that starts on is treated as on. There
is no browse/interact state, CH2 neutral requirement, or off-on rearm latch.

## Safety flow

```text
task_operator request
    -> task_supervisor mode
    -> task_control desired current
    -> safety_balance_control approved current
    -> task_motor CAN current frames
```

Balance requires fresh RC, safe IMU, valid 200 Hz state, active CAN, fresh
feedback from all four actuators, no motor faults, lean inside the arming
window, lean rate inside the arming window, and current within the configured
limit.

A running inhibit starts a zero-current stop window. With SA and SD still on,
the controller automatically arms again when the stop window has elapsed and
every inhibit has cleared. This removes the former `SD OFF-ON` state, but the
operator must turn the switches off before approaching the mechanism.

## Lean zeroing

Press SE on READY while the robot is supported and stationary. A valid request
captures the current raw mounted lean as `lean_trim_rad`. The dashboard then
shows the corrected lean near zero.

Zeroing is rejected when IMU lean is unavailable, balance is active, absolute raw
lean exceeds the configured window, or lean rate is too high. A large upright
error near 90 degrees indicates incorrect IMU orientation and must be corrected
in `config_imu.h`.

## Commissioning order

1. Verify forward tilt, lean rate, wheel position, and wheel velocity signs.
2. Verify SA off, SD off, RC loss, IMU loss, CAN loss, feedback loss, motor
   fault, excessive lean, and excessive current all command zero current.
3. Set position, velocity, and integral gains to zero. Tune lean gain, then
   lean-rate damping on the fixture.
4. Attempt low-authority balancing with a catch frame or tether.
5. Add velocity feedback, then position feedback, using small impulse pushes.
6. Add a small integral gain only after the other loops are stable.
7. Run repeated enable, disturbance, stop, fault, and recovery cycles while
   monitoring current saturation, CAN health, temperature, and control overruns.

## Configuration

All values below are in `platforms/esp32/main/config/config_control.h`.

```text
APP_BALANCE_INITIAL_LEAN_TRIM_RAD
APP_BALANCE_ZERO_MAX_ABS_LEAN_RAD
APP_BALANCE_ZERO_MAX_LEAN_RATE_RPS

APP_BALANCE_OUTPUT_SIGN
APP_BALANCE_LEAN_GAIN_A_PER_RAD
APP_BALANCE_LEAN_RATE_GAIN_A_PER_RPS
APP_BALANCE_POSITION_GAIN_A_PER_M
APP_BALANCE_VELOCITY_GAIN_A_PER_MPS
APP_BALANCE_POSITION_INTEGRAL_GAIN_A_PER_M_S
APP_BALANCE_POSITION_INTEGRAL_LIMIT_M_S
APP_BALANCE_MIX_1 ... APP_BALANCE_MIX_4

APP_BALANCE_MAX_CURRENT_A
APP_BALANCE_REQUEST_MAX_AGE_US
APP_BALANCE_STATE_MAX_AGE_US
APP_BALANCE_STOP_HOLD_US
APP_BALANCE_ARM_LEAN_LIMIT_RAD
APP_BALANCE_ARM_LEAN_RATE_LIMIT_RPS
APP_BALANCE_FALL_LEAN_LIMIT_RAD
```

The checked-in gains are commissioning values, not validated robot gains. The
integral gain is intentionally zero.

## Physical values needed for the model

- loaded wheel radius and wheel locations;
- roller handedness;
- base and body masses;
- center-of-mass height and offset;
- pitch inertia;
- wheel and reflected rotor inertia;
- gear ratio and motor pole pairs;
- measured current-to-output-torque relationship;
- efficiency, static friction, directional friction, and backlash;
- motor current-loop delay and bandwidth;
- IMU delay, noise, bias, and filter response;
- CAN command and feedback latency;
- battery voltage and current capability; and
- safe continuous and peak motor current.
