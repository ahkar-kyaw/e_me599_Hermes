#ifndef SVC_OTA_ESP32_H
#define SVC_OTA_ESP32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SVC_OTA_ESP32_OK = 0,
    SVC_OTA_ESP32_ERROR_INIT = -1,
    SVC_OTA_ESP32_ERROR_WIFI = -2,
    SVC_OTA_ESP32_ERROR_DISCOVERY = -3,
    SVC_OTA_ESP32_ERROR_DOWNLOAD = -4,
    SVC_OTA_ESP32_ERROR_IMAGE = -5
} svc_ota_esp32_result_t;

typedef void (*svc_ota_esp32_progress_fn)(
    uint8_t progress_percent,
    void *context);

svc_ota_esp32_result_t svc_ota_esp32_connect(
    const char *ssid,
    const char *password,
    uint32_t timeout_ms);

svc_ota_esp32_result_t svc_ota_esp32_discover_server(
    uint16_t discovery_port,
    uint32_t timeout_ms,
    char *server_address,
    size_t address_capacity);

svc_ota_esp32_result_t svc_ota_esp32_install_from_server(
    const char *server_address,
    uint16_t server_port,
    const char *firmware_path,
    uint32_t timeout_ms,
    svc_ota_esp32_progress_fn progress_fn,
    void *progress_context);

void svc_ota_esp32_disconnect(void);
bool svc_ota_esp32_is_connected(void);
bool svc_ota_esp32_get_ipv4(char *address, size_t capacity);
bool svc_ota_esp32_confirm_running_image(void);

#ifdef __cplusplus
}
#endif

#endif
