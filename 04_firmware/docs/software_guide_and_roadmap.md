# Software guide and roadmap

## Current application

The current ESP32 application provides:

- ISM330DHCX sampling, gyro calibration, mounting correction, and attitude;
- CRSF reception and safety validation;
- four-actuator CubeMars CAN feedback and servo-current transmission;
- 200 Hz forward-plane state estimation and balance control;
- safety-gated current with freshness, tilt, feedback, fault, and CAN checks;
- a two-screen READY/ACTIVE ST7789 dashboard; and
- runtime lean zeroing from the READY screen.

## Current operating sequence

1. Power the transmitter with SA and SD off.
2. Power the robot while it is supported and keep it stationary for IMU gyro
   calibration.
3. Confirm READY shows RC, IMU, CAN, and motors healthy.
4. Turn SA on.
5. Press SE once to zero the supported upright lean.
6. Move the robot to its restrained ground test position.
7. Turn SD on and confirm `BALANCE LIVE`.
8. Turn SD or SA off to stop.

## Important behavior

- SA and SD are direct switch levels; no startup edge is required.
- There is no off-on rearm latch.
- A transient inhibit causes zero-current stopping. If both switches remain on,
  torque can return automatically after the stop window when all inhibits clear.
- The lean trim is not persisted across reboot.
- The current limit remains 1.0 A until balance tuning justifies an increase.

## Next software milestones

1. Validate signs, zeroing, safety stops, and task timing on the fixture.
2. Tune lean and lean-rate feedback at low current.
3. Add velocity and position recovery, then integral disturbance rejection.
4. Update the physical model and MATLAB parameters; compare measured and model
   response before replacing the commissioning gains.
5. Add controlled forward reference generation only after stationary balance is
   repeatable.
6. Add lateral/yaw behavior after the forward-plane controller is stable.
7. Add parameter persistence only when a safe validation and rollback process
   is defined.
