# OTA update

The Mac and ESP32 join the same phone hotspot. Wi-Fi remains off during normal
robot operation. SC low arms the trigger; an accepted low-to-high transition
connects Wi-Fi, discovers the Mac, downloads the image, validates it, and
reboots.

## Configuration

Use `idf.py menuconfig` under `Application OTA update`. The generated
`sdkconfig.h` symbols are:

```text
CONFIG_APP_OTA_ENABLE
CONFIG_APP_OTA_WIFI_SSID
CONFIG_APP_OTA_WIFI_PASSWORD
CONFIG_APP_OTA_TRIGGER_CHANNEL
```

Do not copy their values into documentation or source. Protocol ports, image
path, and timeouts are defined in `main/config/config_ota.h`. The local
`sdkconfig` containing credentials is ignored by Git.

The phone hotspot must use a 2.4 GHz-compatible mode. Connect the Mac to the
same hotspot.

## Update

Build:

```bash
cd 04_firmware/platforms/esp32
source ~/esp/esp-idf-v5.5/export.sh
idf.py build
```

Run the repository OTA server from `04_firmware`:

```bash
cd ../..
python3 tools/serve_ota.py
```

Allow incoming Python connections through the macOS firewall. With SA and SD
off, move SC from low to high. The display progresses through Wi-Fi connection,
Mac discovery, download, validation, and reboot. The server prints the ESP32
address and HTTP request.

## Failure states

```text
APP_OTA_ERROR_INIT        initialization
APP_OTA_ERROR_WIFI        Wi-Fi connection
APP_OTA_ERROR_DISCOVERY   Mac discovery
APP_OTA_ERROR_DOWNLOAD    HTTP download
APP_OTA_ERROR_IMAGE       image validation
```

`common/include/app/app_ota_types.h` is authoritative for these values.

For discovery failure, confirm that the Mac is on the same hotspot, the server
tool is running, and the firewall permits Python. OTA cannot cross a hotspot
that isolates clients. Move SC low before another attempt.

## Disable

Clear `CONFIG_APP_OTA_ENABLE` through `idf.py menuconfig`, build, and install
that image through OTA. Re-enabling OTA then requires USB flashing.
