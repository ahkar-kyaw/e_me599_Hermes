# CAN transmit buffer lifetime incident

## Summary

During the open-loop actuator drive test, actuators 3 and 4 moved much faster
than actuator 1 while actuator 2 remained stationary. The four logical current
requests shown on the display were equal, so the investigation followed the
command path from safety approval through CAN transmission.

The ESP32 CAN backend passed pointers to stack-owned frame and payload objects
to the asynchronous ESP-IDF TWAI driver. Those objects stopped being valid as
soon as the transmit function returned. Later actuator commands reused the same
stack memory while an earlier CAN frame could still be pending.

This was a firmware defect. It could corrupt or replace queued actuator frames
and create repeatable differences between actuator slots even though their
logical requests were equal.

## Affected implementation

The affected function was:

```text
platforms/esp32/main/bsp/bsp_can_esp32.c
    bsp_can_esp32_transmit()
```

The previous implementation created both of these objects locally:

```c
uint8_t data[IF_CAN_CLASSIC_MAX_DATA_LENGTH];
twai_frame_t twai_frame;
```

`twai_node_transmit()` queues a pointer to the supplied `twai_frame_t`. The
frame and the memory referenced by its `buffer` member must remain valid until
the transmit-complete event.

## Correction

The CAN backend now owns one persistent transmit slot containing:

- a persistent `twai_frame_t`;
- a persistent eight-byte Classic CAN payload buffer; and
- a one-entry FreeRTOS queue that represents transmit-slot availability.

The transmit function takes the slot, fills the persistent frame, and queues
it with ESP-IDF. The TWAI transmit-complete callback returns the slot from ISR
context. A following actuator command cannot overwrite the frame until the
driver reports that the previous transmission is complete.

Only one frame is allowed to be pending. This matches the existing one-entry
TWAI transmit queue, bounds memory use, and prevents a backlog of stale motor
commands. The motor task still sends all four commands every 5 ms and retains
its existing 5 ms timeout.

No control, safety, actuator-ID, direction-sign, or CubeMars protocol behavior
was changed.

## Why equal current does not imply equal speed

The commissioning test uses CubeMars servo-current mode. Current commands set
torque-producing Iq current rather than wheel speed. Equal requested currents
therefore do not guarantee identical unloaded speeds when friction, gearbox
drag, wheel load, or actuator calibration differs.

The firmware also converts robot-coordinate commands to physical motor
directions immediately before CAN encoding:

```text
actuator 1 direction -1
actuator 2 direction -1
actuator 3 direction +1
actuator 4 direction +1
```

For a positive logical forward request, actuators 1 and 2 receive negative raw
current while actuators 3 and 4 receive positive raw current. This is expected
for the mirrored mounting and is independent of the transmit-buffer defect.

## Verification

### Software verification

Run the host tests and ESP-IDF build:

```bash
cd 04_firmware/tests
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure

cd ../platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py build
```

### Fixture verification

1. Secure the robot with every wheel clear of the fixture.
2. Start with SA and SD off and all motion sticks centered.
3. Turn SA on and use SE to zero lean if needed.
4. Turn SD on and wait for `LIVE`.
5. Move CH3 slightly forward. Confirm all four `REQ` values are equal.
6. Move CH3 slightly backward. Confirm all four `REQ` values reverse together.
7. Center CH3 and move CH4. Confirm motors 1 and 2 share one logical sign and
   motors 3 and 4 use the opposite sign.
8. Compare `FB` current values in both directions.
9. Lower SD and confirm immediate zero-current stopping.

Expected behavior is reliable command delivery to all four actuators. Unloaded
wheel speeds may still differ because this is current control rather than speed
control.

If actuator 2 remains stationary after this correction, record `REQ` and `FB`
for both CH3 directions. Equal `REQ` with near-zero actuator-2 `FB` points to
the actuator current-control path or drive configuration. Similar `FB` with no
motion points to mechanical drag, gearbox resistance, or calibration.

## References

- ESP-IDF 5.5 TWAI programming guide:
  <https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32/api-reference/peripherals/twai.html>
- CubeMars AK-series driver software manual:
  <https://www.cubemars.com/data/cms/202610/ak-series-motor-module-v3-driver-software-user-manual-en.pdf>
