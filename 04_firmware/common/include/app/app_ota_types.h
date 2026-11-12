#ifndef APP_OTA_TYPES_H
#define APP_OTA_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_OTA_STATE_IDLE = 0,
    APP_OTA_STATE_WIFI_CONNECTING,
    APP_OTA_STATE_DISCOVERING,
    APP_OTA_STATE_DOWNLOADING,
    APP_OTA_STATE_SUCCESS,
    APP_OTA_STATE_FAILED
} app_ota_state_t;

typedef enum
{
    APP_OTA_ERROR_NONE = 0,
    APP_OTA_ERROR_INIT = -1,
    APP_OTA_ERROR_WIFI = -2,
    APP_OTA_ERROR_DISCOVERY = -3,
    APP_OTA_ERROR_DOWNLOAD = -4,
    APP_OTA_ERROR_IMAGE = -5
} app_ota_error_t;

typedef struct
{
    uint64_t timestamp_us;
    app_ota_error_t error;
    uint8_t progress_percent;
    app_ota_state_t state;
    bool enabled;
    char wifi_ipv4[16];
} app_ota_snapshot_t;

static inline bool app_ota_state_active(app_ota_state_t state)
{
    return (state == APP_OTA_STATE_WIFI_CONNECTING) ||
           (state == APP_OTA_STATE_DISCOVERING) ||
           (state == APP_OTA_STATE_DOWNLOADING) ||
           (state == APP_OTA_STATE_SUCCESS);
}

#ifdef __cplusplus
}
#endif

#endif
