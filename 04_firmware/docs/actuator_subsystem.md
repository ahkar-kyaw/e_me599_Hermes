# Actuator subsystem

## Active configuration

Four CubeMars actuators are configured in
`platforms/esp32/main/config/config_actuator.h`.

```text
actuator 1 -> CAN ID 1 -> direction -1
actuator 2 -> CAN ID 2 -> direction -1
actuator 3 -> CAN ID 3 -> direction +1
actuator 4 -> CAN ID 4 -> direction +1
```

Physical order, using the paper's right-positive axle coordinate:

```text
actuator 1 -> +95 mm -> far right
actuator 2 -> +33 mm -> inner right
actuator 3 -> -33 mm -> inner left
actuator 4 -> -95 mm -> far left
```

Direction signs convert physical actuator mounting into robot coordinates.
Controller gains and balance mix values should not be made negative to repair a
single incorrect motor direction.

## Active command mode

The application transmits only CubeMars servo-current commands. The earlier
single-motor velocity and RC manual-current commissioning paths were removed.
The protocol module may retain encoders for other documented actuator modes,
but the motor task accepts only safety-approved current.

## Current limits

The actuator driver settings reported for the current hardware are:

```text
motor current range       -2.7 A to +2.7 A
absolute motor maximum     4.1 A
battery source maximum     3.0 A
battery regeneration min  -1.25 A
speed controller Kp        0.0005
speed controller Ki        0.08
position controller Kp     0.01
position controller Ki     0
position controller Kd     0
```

The initial firmware balance limit is `APP_BALANCE_MAX_CURRENT_A = 1.0 A`.
It must remain below the configured 2.7 A driver limit. The
speed and position gains are internal drive settings and are not used by the
firmware while servo-current mode is active.

## Safety behavior

Motion requires an active CAN bus and fresh, configured, fault-free feedback
from every actuator. A fault or stale command starts a zero-current stop
window and enters `LOCKOUT`. Torque cannot return until the operator turns SD
or SA off, clears the fault, centers the motion sticks, and arms again.

## Incident records

- [CAN transmit buffer lifetime correction](can_transmit_buffer_lifetime_fix.md)
