# Firmware reference

Start with [../README.md](../README.md) for architecture, controls, build, and
operating modes. Open only the subsystem document needed for the task.

| Document | Use it for |
|---|---|
| [ota_update.md](ota_update.md) | Configure and run wireless updates |
| [balance_control.md](balance_control.md) | Control law, signs, limits, and tuning order |
| [state_estimation_and_control.md](state_estimation_and_control.md) | Estimator equations and state definitions |
| [robot_configuration_checklist.md](robot_configuration_checklist.md) | Values that must match the assembled robot |
| [actuator_subsystem.md](actuator_subsystem.md) | Motor IDs, directions, CAN, and current limits |
| [imu_subsystem.md](imu_subsystem.md) | IMU axes, estimator, and calibration |
| [ui_subsystem.md](ui_subsystem.md) | READY, ACTIVE, and UPDATE screens |
| [electrical_interfaces.md](electrical_interfaces.md) | Firmware pin and connector assumptions |
| [firmware_architecture.md](firmware_architecture.md) | Task ownership and command flow |
| [naming_conventions.md](naming_conventions.md) | File, type, function, and prefix rules |
| [software_guide_and_roadmap.md](software_guide_and_roadmap.md) | Final operating and thesis validation checklist |

`can_transmit_buffer_lifetime_fix.md` is an incident record retained for the
thesis and regression history.
