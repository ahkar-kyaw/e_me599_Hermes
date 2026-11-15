# State estimation and control

## State and reference flow

The periodic control task combines mount-corrected IMU attitude with four-wheel
feedback. It publishes timestamped position, velocity, yaw, lean, and lean-rate
state with an explicit validity flag.

RC motion inputs become bounded forward velocity, lateral velocity, and yaw
rate references. References are slew-limited. When a stick returns to center,
the corresponding bounded position or yaw reference stops moving, allowing the
controller to resist small external displacement.

Lean presented to control is:

```text
lean = APP_CONTROL_LEAN_SIGN * mounted_raw_lean - active_lean_trim
```

An accepted SE press captures the runtime trim while the robot is stationary in
READY. A reboot restores `APP_BALANCE_INITIAL_LEAN_TRIM_RAD`. Mounting errors
must be corrected in `config_imu.h`, not hidden with trim.

## Controller

The implemented axis controller has the form:

```text
current = output_sign * (
      Klean * lean
    + Krate * lean_rate
    - Kpos  * position_error
    - Kvel  * velocity
    - Ki    * integral(position_error))
```

Forward control includes lean stabilization. Lateral and yaw use bounded
position/velocity and angle/rate feedback. Axis limits, reference leashes,
integrator bounds, and final common motor scaling prevent one requested axis
from silently changing the requested motion combination during saturation.

The controller produces desired current only. `task_safety` is the sole owner
of motion permission and publishes the only command accepted by `task_motor`.

## Collinear mecanum model

Estimation and allocation use Equation (8) from Watson, Gladwin, and Prescott,
*The Collinear Mecanum Drive: Modelling, Analysis, Partial Feedback
Linearisation, and Nonlinear Control*:

```text
r_w * omega_i = -cot(alpha_i) * v_body_x
                + v_body_y
                + l_i * yaw_rate
```

The paper frame is used directly for odometry: body `+x` follows the wheel axis,
body `+y` is forward, and positive yaw is counterclockwise. At zero yaw these
body axes align with inertial `+x` and `+y`. The estimator applies the
pseudoinverse of the wheel map. Application motion commands remain named
`forward` and `left`, where `v_forward = v_body_y` and
`v_left = -v_body_x`.

Configured geometry follows the paper's wheel order:

| Wheel | `l_i` (m) | `alpha_i` |
|---:|---:|---:|
| 1 | -0.095 | +45 deg |
| 2 | -0.033 | -45 deg |
| 3 | +0.033 | +45 deg |
| 4 | +0.095 | -45 deg |

`APP_CONTROL_LEAN_SIGN` converts the mounted IMU pitch axis to the paper's
positive rotation about body `+x`. It changes lean angle and lean rate together.

The current controller follows the paper's nested stabilization and bounded
motion structure; it does not claim the full nonlinear
partial-feedback-linearization/backstepping law. That requires a validated
physical model.

- [Accepted paper](https://eprints.whiterose.ac.uk/id/eprint/157762/1/journal_backstepping.pdf)
- [Author reference code](https://github.com/matthew-t-watson/CollinearMecanumDrive)

## Tuning order

All authoritative values are in `config_control.h`.

1. Verify geometry, feedback conversion, roller handedness, and all signs.
2. Tune lean gain, then lean-rate damping.
3. Tune forward velocity, then forward position.
4. Tune lateral velocity, then lateral position.
5. Tune yaw rate, then yaw position.
6. Set motion speed and acceleration limits.
7. Set position-error leashes and add integral action only if a measured steady
   bias remains.
8. Increase axis and global current limits only after thermal, electrical, and
   mechanical validation.

Each stage must preserve arm, stop, RC-loss, IMU-loss, CAN-loss, stale-feedback,
motor-fault, excessive-lean, and current-limit behavior.
