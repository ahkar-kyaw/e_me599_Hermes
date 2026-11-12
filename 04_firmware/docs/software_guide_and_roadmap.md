# Final validation checklist

## Operating check

1. Power the transmitter with SA and SD off.
2. Power the robot while it is supported and keep it stationary for IMU gyro
   calibration.
3. Confirm READY shows RC, IMU, CAN, and motors healthy.
4. Turn SA on.
5. Press SE once to zero the supported upright lean.
6. Move the robot to its restrained ground test position.
7. Center CH1, CH3, and CH4, then turn SD on. Confirm `WAIT SAFE` changes to
   `LIVE` after 300 ms.
8. Use CH3 for forward/reverse, CH1 for lateral motion, and CH4 for yaw.
9. Turn SD or SA off to stop.

## Required behavior

- SA and SD are direct switch levels; no startup edge is required.
- A running fault causes zero-current stopping and then `LOCKOUT`; torque does
  not automatically return. Turn SD or SA off before rearming.
- Motion sticks must be centered during the 300 ms arming dwell.
- The lean trim is not persisted across reboot.
- The current limit remains 1.0 A until balance tuning justifies an increase.
- OTA requires SA and SD off and a deliberate SC low-to-high transition.

## Before thesis sign-off

1. Validate geometry, body-axis signs, zeroing, every stop path, and task timing.
2. Tune lean and lean-rate feedback, then forward velocity and position.
3. Tune lateral velocity/position and yaw rate/angle independently.
4. Measure the completed robot and update the MATLAB model before adopting a
   model-based controller or simulated gains.
5. Add small integral gains only if repeatable steady-state bias remains.
6. Run disturbance, thermal, battery, RC-loss, CAN-loss, and endurance tests.
7. Add parameter persistence only with range validation and a safe rollback.
