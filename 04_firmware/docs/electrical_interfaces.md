# Electrical interfaces

`platforms/esp32/include/board_esp32_nodemcu_v1.h` is the authoritative
firmware pin map. Update the board header and PCB documentation together.

## Pin map

| Interface | Signal | ESP32 pin |
|---|---|---|
| IMU SPI | SCLK / MISO / MOSI / CS | GPIO18 / GPIO19 / GPIO23 / GPIO27 |
| IMU interrupt | INT1 / INT2 | GPIO34 / GPIO35; currently unused |
| LCD SPI | SCLK / MOSI / CS / D-C / RESET | GPIO14 / GPIO13 / GPIO25 / GPIO21 / GPIO22 |
| CRSF UART2 | RX / TX | GPIO16 / GPIO17 |
| CAN TWAI | TX / RX | GPIO26 / GPIO32 |

## ISM330DHCX

- SPI mode and clock are defined in the board header.
- VDD, VDDIO, and logic are 3.3 V with a common ground.
- Firmware currently polls; interrupt pins are reserved.
- Keep the IMU supply, ground, and SPI routing away from motor-current paths and
  switching nodes.

## CubeMars CAN bus

- Four AK40-10 actuators use classic CAN through the TJA1051T/3 transceiver.
- Bitrate and actuator IDs are authoritative in `config_actuator.h`.
- CAN uses one linear trunk, short drops, and exactly two endpoint 120-ohm
  terminations. Do not create a star or enable termination at intermediate
  nodes.
- CANH, CANL, and CAN ground must be documented on the harness. Do not infer
  connector order from wire color.
- The CAN interface is not isolated. Logic, transceiver, and actuator CAN
  grounds must share a deliberate reference.

Motor power is separate from logic power. Document pack voltage, fuse and
branch protection, hardware power disconnect, precharge/inrush behavior, wire
and connector ratings, return-current paths, and regenerative-energy handling.

## ExpressLRS receiver

- CRSF uses UART2 at the bitrate defined in the board header.
- RP2 TX connects to ESP32 RX; RP2 RX connects to ESP32 TX.
- Receiver supply and logic levels must follow the installed receiver/module
  specification, with common ground.
- Keep the receiver and antenna away from motor phase wiring, battery paths,
  switching regulators, and the CAN transceiver.

## ST7789 LCD

- The Waveshare display uses four-wire SPI with no MISO connection.
- Display supply and logic are 3.3 V with common ground.
- Backlight is always on; no firmware backlight GPIO is assigned.
- Mounting orientation is configured in `config_ui.h`.
- Keep the display harness short and prevent its return current from sharing the
  IMU ground path.

## Battery and E-stop

Battery sensing and firmware-visible E-stop inputs are not assigned in the
current board header. When added, document ADC scaling, protection, filtering,
active level, pulls, connector pins, and firmware reaction. Hardware emergency
power removal must not depend on firmware.
