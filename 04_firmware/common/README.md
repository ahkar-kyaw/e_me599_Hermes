# Portable common code

`common` contains code that can be built without an MCU SDK, RTOS headers,
MCU pins, or board headers.

## Modules

```text
app/
    shared snapshots, requests, state, commands, and operator mapping

protocols/
    proto_crsf
    proto_cubemars_ak

drivers/
    drv_ism330dhcx
    drv_st7789

estimation/
    est_imu_mount
    est_imu_calibration
    est_attitude
    est_collinear_mecanum

control/
    ctrl_balance_controller

safety/
    safety_rc
    safety_imu
    safety_balance_control

ui/
    ui_canvas
    ui_dashboard

interfaces/
    if_spi
    if_can
    if_display_io
```

## Boundaries

- Protocol modules only encode and decode bytes.
- Drivers own device behavior and depend on portable interfaces.
- Estimation consumes snapshots and produces state.
- Control computes desired current but does not grant motion permission.
- Safety validates freshness, faults, limits, and operating state.
- Application logic maps operator controls independently of display timing.
- UI formats read-only snapshots; it does not request modes or command motors.
- Platform tasks own RTOS scheduling and hardware integration.

The removed manual velocity, manual current, multi-page navigation, and
browse/interact modules were commissioning-only paths and are not part of the
current balance application.
