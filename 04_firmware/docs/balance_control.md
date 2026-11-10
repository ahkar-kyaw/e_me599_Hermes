# Balance and disturbance-rejection control

## Implemented scope

The active controller stabilizes the robot in the forward balance plane. It
uses pitch, pitch rate, forward wheel position, forward velocity, and a bounded
forward-position integral. It commands a common current which is distributed
to all four actuators through configurable balance mix values.

This milestone does not yet control lateral position or yaw. Those axes should
be added only after the physical model is updated and the single-axis balance
loop is validated.

The portable controller is:

```text
ctrl_balance_controller
    captures the current forward position when balance mode is requested
    computes lean and position errors at 200 Hz
    applies lean, lean-rate, position, velocity, and integral feedback
    clamps the common current and limits integral windup
    contains no ESP-IDF, FreeRTOS, CAN, board, or motor-driver code
```

The implemented control law is:

```text
position_error = position_x - captured_position_x
lean_error = lean - lean_trim

common_current = output_sign * (
      lean_gain      * lean_error
    + lean_rate_gain * lean_rate
    - position_gain  * position_error
    - velocity_gain  * velocity_x
    - integral_gain  * integral(position_error))
```

The output is limited by `APP_CONTROL_TORQUE_MAX_CURRENT_A`. Per-actuator
direction signs remain in `config_actuator.h`; the controller operates only in
robot coordinates.

## Ownership and safety flow

```text
CONTROL page, selection 4, SD Interact request
    |
    v
task_supervisor -> APP_SYSTEM_MODE_BALANCE_CONTROL
    |
    v
task_control -> ctrl_balance_controller -> desired current snapshot
    |
    v
task_safety -> safety_torque_control -> approved current snapshot
    |
    v
task_motor -> CubeMars servo-current CAN frames
```

Balance mode requires:

```text
SC input gate enabled
CONTROL selection 4 active
SD Interact request active
CH2 centered and kept centered
valid and fresh RC
valid and fresh IMU state
valid 200 Hz control state
active CAN bus
fresh, configured, fault-free feedback from all four actuators
lean and lean-rate inside the arming limits
every desired current inside the configured current limit
```

Dropping SC or SD, leaving the page, moving CH2, losing a required snapshot,
CAN state degradation, a motor fault, excessive lean, or invalid controller
output initiates zero-current stopping. A running hard fault requires SD to be
cycled before balance can arm again.

## UI operation

```text
1. Keep SC and SD off and center CH2.
2. Enable SC and browse to CONTROL.
3. Use CH2 in Browse mode to select BALANCE 4/4.
4. Return CH2 to center.
5. Enable SD.
6. Confirm BALANCE LIVE before applying a disturbance.
7. Disable SD or SC to stop.
```

The BALANCE view shows lean error, lean rate, position error, position
integral, common current, current saturation, safety state, and the primary
inhibit cause.

## Required commissioning tests

Perform the tests in order. Do not proceed when a sign, limit, freshness check,
or stop behavior is wrong.

### 1. State and output signs on the fixture

Keep the current limit low. Verify:

```text
forward tilt       positive lean
forward lean rate  positive lean rate
forward wheel move positive position and velocity
forward lean       positive corrective common current
forward position   negative return current when upright
```

If all actuator reactions are reversed, change `APP_BALANCE_OUTPUT_SIGN`.
If only one actuator is reversed, correct its
`APP_ACTUATOR_n_DIRECTION_SIGN`. Do not repair a mounting or motor sign error
by making feedback gains negative.

### 2. Safety and stop behavior

While restrained, test each condition separately:

```text
SD off
SC off
CH2 moved away from center
RC transmitter off
one actuator feedback disconnected
CAN bus interrupted
one motor fault reported
IMU made stale or invalid
lean beyond the running fall limit
```

Each test must produce zero-current stopping. Hard faults must require SD to
be cycled before rearming.

### 3. Lean feedback on the fixture

Temporarily set position, velocity, and integral gains to zero. Increase the
lean gain only until the actuator response clearly opposes lean. Add lean-rate
gain until the response is damped rather than oscillatory. Confirm the command
does not spend significant time at the current limit.

### 4. Low-authority balancing

Use a catch frame or tether. Begin with a small lean window and low current
limit. Confirm the robot catches small lean errors in both directions. Record
lean, lean rate, current, task overruns, and motor faults before increasing
authority.

### 5. Position recovery

Add velocity feedback first, then position feedback. Apply small impulse pushes
in the balance direction. The robot should arrest velocity and return toward
the captured position without sustained oscillation.

### 6. Constant-disturbance rejection

Start with integral gain zero. Add a small integral gain only after the other
loops are stable. Apply a light steady force and confirm the residual position
error decreases without slow oscillation or long current saturation. Keep the
integral limit small enough that releasing the force does not cause a large
rebound.

### 7. Endurance and timing

Run repeated arm, disturbance, stop, and rearm cycles. Confirm no 200 Hz
control overruns, stale snapshots, CAN errors, rising motor temperature, or
unbounded position integral.

## Configuration variables

All balance-loop values are in:

```text
04_firmware/platforms/esp32/main/config/config_control.h
```

```text
APP_BALANCE_OUTPUT_SIGN
    Whole-controller output polarity. Must be +1 or -1.

APP_BALANCE_LEAN_TRIM_RAD
    Upright mechanical trim after the IMU orientation is correct.

APP_BALANCE_LEAN_GAIN_A_PER_RAD
    Restoring current from lean angle.

APP_BALANCE_LEAN_RATE_GAIN_A_PER_RPS
    Damping current from lean angular rate.

APP_BALANCE_POSITION_GAIN_A_PER_M
    Return-to-reference current from forward position error.

APP_BALANCE_VELOCITY_GAIN_A_PER_MPS
    Forward velocity damping.

APP_BALANCE_POSITION_INTEGRAL_GAIN_A_PER_M_S
    Rejection of constant forward disturbances and bias.

APP_BALANCE_POSITION_INTEGRAL_LIMIT_M_S
    Integral windup bound.

APP_BALANCE_MIX_1 ... APP_BALANCE_MIX_4
    Per-actuator scale applied before motor direction signs.

APP_CONTROL_TORQUE_MAX_CURRENT_A
    Hard per-actuator current limit shared with safety.

APP_CONTROL_ARM_LEAN_LIMIT_RAD
APP_CONTROL_ARM_LEAN_RATE_LIMIT_RPS
APP_CONTROL_FALL_LEAN_LIMIT_RAD
    Balance arming and running safety envelope.
```

The checked-in controller gains are conservative commissioning values, not
validated robot gains. The integral gain is intentionally zero until the lean,
lean-rate, velocity, and position feedback are stable. Replace the gains using
measured hardware response and the updated MATLAB model.

## Physical model values to update

Before transferring model-derived gains, update:

```text
loaded wheel radius
wheel locations and roller handedness
body and base masses
center-of-mass height and offset from the axle
body pitch inertia about its center of mass
wheel and reflected rotor inertia
gear ratio and motor pole-pair count
motor current-to-output-torque relationship
gearbox efficiency and directional friction
drivetrain backlash and static friction
motor current-loop delay and bandwidth
IMU delay, noise, bias, and filter response
CAN command and feedback latency
battery voltage range and current capability
safe continuous and peak motor current
```

After those values are updated, the MATLAB model can generate a higher-quality
state-feedback design. Keep the same firmware state order and safety-approved
command boundary when replacing the initial controller.
