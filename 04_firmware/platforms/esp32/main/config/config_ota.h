#ifndef CONFIG_OTA_H
#define CONFIG_OTA_H

#include "sdkconfig.h"

#ifdef CONFIG_APP_OTA_ENABLE
#define APP_OTA_ENABLED                1
#define APP_OTA_WIFI_SSID              CONFIG_APP_OTA_WIFI_SSID
#define APP_OTA_WIFI_PASSWORD          CONFIG_APP_OTA_WIFI_PASSWORD
#define APP_OTA_TRIGGER_CHANNEL        CONFIG_APP_OTA_TRIGGER_CHANNEL
#else
#define APP_OTA_ENABLED                0
#define APP_OTA_WIFI_SSID              ""
#define APP_OTA_WIFI_PASSWORD          ""
#define APP_OTA_TRIGGER_CHANNEL        10
#endif

#define APP_OTA_SERVER_PORT            8000u
#define APP_OTA_DISCOVERY_PORT         3232u
#define APP_OTA_FIRMWARE_PATH          "/app_esp32.bin"
#define APP_OTA_TRIGGER_OFF_THRESHOLD  600u
#define APP_OTA_TRIGGER_ON_THRESHOLD   1384u
#define APP_OTA_CONNECT_TIMEOUT_MS     15000u
#define APP_OTA_DISCOVERY_TIMEOUT_MS    5000u
#define APP_OTA_HTTP_TIMEOUT_MS        15000u
#define APP_OTA_CONFIRM_DELAY_MS       10000u
#define APP_OTA_REBOOT_DELAY_MS         1000u
#define APP_OTA_TASK_PERIOD_MS            50u

#endif
