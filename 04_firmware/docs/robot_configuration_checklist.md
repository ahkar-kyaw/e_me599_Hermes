# Robot configuration checklist

This file is the handoff list for values that must be confirmed from the
assembled robot. The current MATLAB model is useful for structure and sign
conventions, but its numerical parameters must not be copied into control gains
until they are remeasured.

## Required before fixture current testing

### IMU mounting and attitude

File:

```text
04_firmware/platforms/esp32/main/config/config_imu.h
```

Confirm or update:

```text
APP_IMU_MOUNT_FORWARD_AXIS       physical sensor axis pointing forward
APP_IMU_MOUNT_UP_AXIS            physical sensor axis pointing up
APP_IMU_ACCEL_ODR                required accelerometer update rate
APP_IMU_GYRO_ODR                 required gyro update rate
APP_IMU_ACCEL_FS                 measured shock range without saturation
APP_IMU_GYRO_FS                  measured fall-rate range without saturation
APP_IMU_COMPLEMENTARY_ALPHA      attitude bandwidth after log-based tuning
APP_IMU_MAX_AGE_US               maximum accepted IMU snapshot age
APP_IMU_MAX_ABS_ROLL_RAD         global IMU safety envelope
APP_IMU_MAX_ABS_PITCH_RAD        global IMU safety envelope
APP_IMU_MIN_ACCEL_NORM_MPS2      acceptable acceleration-norm lower limit
APP_IMU_MAX_ACCEL_NORM_MPS2      acceptable acceleration-norm upper limit
```

Expected body-frame checks for the current configuration:

```text
forward tilt    positive pitch
left tilt       negative roll
left yaw        positive yaw rate
stationary      body Z acceleration near +9.81 m/s^2
```

### Actuator and CAN setup

File:

```text
04_firmware/platforms/esp32/main/config/config_actuator.h
```

Confirm or update:

```text
APP_ACTUATOR_CAN_BITRATE                 actuator bus bitrate
APP_ACTUATOR_n_CAN_ID                    physical motor-to-slot mapping
APP_ACTUATOR_n_DIRECTION_SIGN            robot-positive command and feedback sign
APP_ACTUATOR_n_ENABLED                   installed actuator slots
APP_ACTUATOR_FEEDBACK_MAX_AGE_US         from configured feedback rate
APP_ACTUATOR_COMMAND_PERIOD_MS            actuator CAN command rate
APP_ACTUATOR_COMMAND_MAX_AGE_US           local command freshness limit
APP_ACTUATOR_COMMAND_STOP_HOLD_US         zero-command transmission time
```

Also set in the CubeMars configuration tool:

```text
unique CAN IDs 1, 2, 3, and 4
1 Mbit/s CAN
servo control mode
periodic feedback rate, initially 100 Hz
CAN command timeout, about 100 ms or less
zero or otherwise deliberately chosen timeout brake current
motor current and input-current limits appropriate for the fixture and supply
```

The actuator-side timeout is independent of firmware. Verify its behavior by
removing CAN commands while the robot remains restrained.

### Fixture current limits and safety envelope

File:

```text
04_firmware/platforms/esp32/main/config/config_control.h
```

Confirm or update:

```text
APP_CONTROL_TORQUE_INPUT_CHANNEL          RC channel used for current
APP_CONTROL_TORQUE_INPUT_POSITIVE_HIGH    stick polarity
APP_CONTROL_TORQUE_MAX_CURRENT_A           fixture-tested current limit
APP_CONTROL_TORQUE_SLEW_RATE_A_PER_S       current ramp limit
APP_CONTROL_TORQUE_MIX_1 ... _4            per-motor command sign and scale
APP_CONTROL_REQUEST_MAX_AGE_US             UI/supervisor request freshness
APP_CONTROL_STATE_MAX_AGE_US               state freshness
APP_CONTROL_STOP_HOLD_US                   safety zero-current hold
APP_CONTROL_ARM_LEAN_LIMIT_RAD             maximum lean when arming
APP_CONTROL_ARM_LEAN_RATE_LIMIT_RPS        maximum lean rate when arming
APP_CONTROL_FALL_LEAN_LIMIT_RAD            running fall cutoff
APP_CONTROL_LEAN_AXIS                      roll or pitch balance axis
```

Start with the checked-in 1.0 A limit. Increase only after verifying current,
direction, temperature, supply behavior, and mechanical reaction.

### RC and UI mapping

Files:

```text
04_firmware/platforms/esp32/main/config/config_rc.h
04_firmware/platforms/esp32/main/config/config_ui.h
```

Confirm or update:

```text
APP_RC_ACTIVE_CHANNEL_COUNT
APP_RC_MAX_AGE_US
APP_UI_RC_*_CHANNEL
APP_UI_RC_*_THRESHOLD
APP_UI_RC_*_HIGH polarity values
APP_DISPLAY_ORIENTATION
```

If EdgeTX endpoints change, record the observed raw low, center, and high
values before adjusting neutral bands or activation thresholds.

## Required before closed-loop balance control

Measure and record these values on the completed robot:

```text
total robot mass
sprung body mass and wheel/rotor masses
center-of-mass height above the wheel axis
body pitch moment of inertia about the wheel axis
wheel radius under load
wheel lateral positions and roller geometry
wheel and reflected rotor inertia
gear ratio and pole-pair count
motor torque constant or measured current-to-output-torque relationship
gearbox and drivetrain efficiency in both directions
static friction, rolling resistance, viscous drag, and backlash/dead zone
maximum safe motor current, battery current, speed, and temperature
battery voltage range and source impedance
control-path delay and actuator current-loop response
IMU noise, bias stability, and effective delay
CAN feedback rate, command rate, and worst-case latency
```

Then update the MATLAB model and identify the plant before choosing controller
gains. Record the state ordering, units, sample time, equilibrium, actuator
saturation, and any integral or disturbance states used by the design.

Controller-specific values should be added only with the controller module:

```text
state-feedback or LQR gains
Q and R design weights
reference-command limits and slew rates
integrator limits and anti-windup behavior
observer or filter gains
disturbance-rejection limits
closed-loop arm, fall, and recovery thresholds
```

Keep these parameters in a portable control configuration structure. Platform
configuration may select values, but common control code must not include
ESP-IDF, FreeRTOS, board pins, or CubeMars CAN formatting.
