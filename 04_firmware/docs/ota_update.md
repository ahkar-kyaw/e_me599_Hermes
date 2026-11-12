# Local OTA update

## Design

```text
Phone hotspot     provides the local Wi-Fi network
serve_ota.py      serves the image and answers UDP discovery
task_ota          owns the SC trigger, state, progress, and rollback check
svc_ota_esp32     owns Wi-Fi, server discovery, HTTP, and installation
partitions.csv    provides two 960 KiB application slots
```

Wi-Fi remains off during normal operation. With SC initially low, a low-to-high
transition while the robot is disarmed enters UPDATE. The ESP32 connects to the
phone hotspot, discovers the Mac automatically, downloads the image, validates
it, and reboots.

## One-time setup

Configure the phone hotspot as:

```text
SSID       Phone
Band       2.4 GHz
Security   WPA2 or WPA2/WPA3 compatibility mode
```

Connect the Mac to that hotspot. Configure the same SSID and password in:

```bash
cd 04_firmware/platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py menuconfig
```

Under `Application OTA update`, enable OTA, enter the hotspot credentials, and
leave the trigger channel at 10 for SC. Install the OTA partition table over
USB once:

```bash
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash
```

## Update workflow

Connect the Mac to `Hermes_OTA`, then build:

```bash
cd 04_firmware/platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py build
```

Start the combined discovery and HTTP server from `04_firmware`:

```bash
cd ../..
python3 tools/serve_ota.py
```

Allow incoming Python connections if the macOS firewall asks. The tool serves
`platforms/esp32/build/app_esp32.bin` on TCP port 8000 and answers discovery on
UDP port 3232.

With SA and SD off, move SC from low to high. Expected display sequence:

```text
CONNECTING TO WIFI
FINDING MAC SERVER
DOWNLOADING n%
UPDATE COMPLETE
```

The tool prints the ESP32 address when discovery succeeds and logs the HTTP
request. Keep it running until the robot reboots; stop it with `Ctrl-C`.

## Failure codes

```text
-1  OTA initialization failed
-2  phone hotspot connection failed
-3  Mac server was not discovered
-4  firmware download failed
-5  firmware image validation failed
```

For `-2`, verify the hotspot name, password, and 2.4 GHz compatibility. For
`-3`, confirm the Mac is on the same hotspot, `serve_ota.py` is running, and the
firewall permits Python. Some phone hotspots isolate clients; OTA cannot work
through a hotspot that prevents the ESP32 and Mac from communicating.

Move SC low to clear a failed update and arm the next attempt.

## Disabling OTA

Clear `Enable RC-triggered local OTA` in `idf.py menuconfig`, build, and install
that image through OTA. Re-enabling OTA then requires a USB flash.
