#include "task_ota.h"

#include "config_ota.h"
#include "svc_ota_esp32.h"
#include "task_operator.h"
#include "task_rc.h"
#include "task_safety.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <string.h>

#define TASK_OTA_STACK_BYTES 8192u
#define TASK_OTA_PRIORITY    (tskIDLE_PRIORITY + 1)

_Static_assert((APP_OTA_TRIGGER_CHANNEL >= 1) &&
               (APP_OTA_TRIGGER_CHANNEL <= APP_RC_CHANNEL_COUNT),
               "OTA trigger channel is outside the CRSF channel range");

static const char *TAG = "task_ota";

static app_ota_snapshot_t g_ota_snapshot;
static portMUX_TYPE g_ota_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;

static void task_ota_entry(void *argument);
static void task_ota_publish(app_ota_state_t state,
                             uint8_t progress_percent,
                             app_ota_error_t error);
static bool task_ota_robot_disarmed(
    const app_operator_controls_state_t *controls,
    const app_balance_safety_snapshot_t *safety);
static void task_ota_progress(uint8_t progress_percent, void *context);
static void task_ota_run_update(void);
static app_ota_error_t task_ota_error(
    svc_ota_esp32_result_t result);

void task_ota_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_ota_entry,
        "task_ota",
        TASK_OTA_STACK_BYTES,
        NULL,
        TASK_OTA_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_ota_get_snapshot(app_ota_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_ota_snapshot_lock);
    *snapshot = g_ota_snapshot;
    portEXIT_CRITICAL(&g_ota_snapshot_lock);
}

bool task_ota_is_active(void)
{
    bool active;

    portENTER_CRITICAL(&g_ota_snapshot_lock);
    active = app_ota_state_active(g_ota_snapshot.state);
    portEXIT_CRITICAL(&g_ota_snapshot_lock);
    return active;
}

static void task_ota_entry(void *argument)
{
    (void)argument;

    task_ota_publish(APP_OTA_STATE_IDLE, 0u, APP_OTA_ERROR_NONE);
    vTaskDelay(pdMS_TO_TICKS(APP_OTA_CONFIRM_DELAY_MS));

    if (!svc_ota_esp32_confirm_running_image())
    {
        ESP_LOGE(TAG, "failed to confirm running OTA image");
    }

    if (APP_OTA_ENABLED == 0)
    {
        vTaskDelete(NULL);
        return;
    }

    bool trigger_armed = false;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        rc_snapshot_t rc_snapshot;
        app_operator_controls_state_t controls;
        app_balance_safety_snapshot_t safety;
        app_ota_snapshot_t ota;

        task_rc_get_snapshot(&rc_snapshot);
        task_operator_get_controls_state(&controls);
        task_safety_get_balance_snapshot(&safety);
        task_ota_get_snapshot(&ota);

        const uint16_t trigger = rc_snapshot.channel[
            APP_OTA_TRIGGER_CHANNEL - 1u];

        if (trigger <= APP_OTA_TRIGGER_OFF_THRESHOLD)
        {
            trigger_armed = controls.link_usable;

            if (ota.state == APP_OTA_STATE_FAILED)
            {
                task_ota_publish(APP_OTA_STATE_IDLE,
                                 0u,
                                 APP_OTA_ERROR_NONE);
            }
        }
        else if ((trigger >= APP_OTA_TRIGGER_ON_THRESHOLD) &&
                 trigger_armed)
        {
            trigger_armed = false;

            if (task_ota_robot_disarmed(&controls, &safety) &&
                (ota.state == APP_OTA_STATE_IDLE))
            {
                task_ota_run_update();
                last_wake = xTaskGetTickCount();
            }
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_OTA_TASK_PERIOD_MS));
    }
}

static void task_ota_publish(app_ota_state_t state,
                             uint8_t progress_percent,
                             app_ota_error_t error)
{
    app_ota_snapshot_t snapshot =
    {
        .timestamp_us = (uint64_t)esp_timer_get_time(),
        .error = error,
        .progress_percent = progress_percent,
        .state = state,
        .enabled = (APP_OTA_ENABLED != 0),
    };

    (void)svc_ota_esp32_get_ipv4(snapshot.wifi_ipv4,
                                 sizeof(snapshot.wifi_ipv4));

    portENTER_CRITICAL(&g_ota_snapshot_lock);
    g_ota_snapshot = snapshot;
    portEXIT_CRITICAL(&g_ota_snapshot_lock);
}

static bool task_ota_robot_disarmed(
    const app_operator_controls_state_t *controls,
    const app_balance_safety_snapshot_t *safety)
{
    return controls->link_usable &&
           !controls->master_enabled &&
           !controls->balance_enabled &&
           (safety->state == APP_BALANCE_SAFETY_DISABLED) &&
           !safety->transmit_enabled &&
           !safety->motion_allowed;
}

static void task_ota_progress(uint8_t progress_percent, void *context)
{
    (void)context;
    task_ota_publish(APP_OTA_STATE_DOWNLOADING,
                     progress_percent,
                     APP_OTA_ERROR_NONE);
}

static void task_ota_run_update(void)
{
    char server_address[16];

    task_ota_publish(APP_OTA_STATE_WIFI_CONNECTING,
                     0u,
                     APP_OTA_ERROR_NONE);
    svc_ota_esp32_result_t result = svc_ota_esp32_connect(
        APP_OTA_WIFI_SSID,
        APP_OTA_WIFI_PASSWORD,
        APP_OTA_CONNECT_TIMEOUT_MS);

    if (result == SVC_OTA_ESP32_OK)
    {
        task_ota_publish(APP_OTA_STATE_DISCOVERING,
                         0u,
                         APP_OTA_ERROR_NONE);
        result = svc_ota_esp32_discover_server(
            APP_OTA_DISCOVERY_PORT,
            APP_OTA_DISCOVERY_TIMEOUT_MS,
            server_address,
            sizeof(server_address));
    }

    if (result == SVC_OTA_ESP32_OK)
    {
        task_ota_publish(APP_OTA_STATE_DOWNLOADING,
                         0u,
                         APP_OTA_ERROR_NONE);
        result = svc_ota_esp32_install_from_server(
            server_address,
            APP_OTA_SERVER_PORT,
            APP_OTA_FIRMWARE_PATH,
            APP_OTA_HTTP_TIMEOUT_MS,
            task_ota_progress,
            NULL);
    }

    svc_ota_esp32_disconnect();

    if (result != SVC_OTA_ESP32_OK)
    {
        task_ota_publish(APP_OTA_STATE_FAILED,
                         0u,
                         task_ota_error(result));
        ESP_LOGE(TAG, "update failed: %ld", (long)result);
        return;
    }

    task_ota_publish(APP_OTA_STATE_SUCCESS,
                     100u,
                     APP_OTA_ERROR_NONE);
    ESP_LOGI(TAG, "update installed; restarting");
    vTaskDelay(pdMS_TO_TICKS(APP_OTA_REBOOT_DELAY_MS));
    esp_restart();
}

static app_ota_error_t task_ota_error(
    svc_ota_esp32_result_t result)
{
    switch (result)
    {
        case SVC_OTA_ESP32_ERROR_WIFI:
            return APP_OTA_ERROR_WIFI;

        case SVC_OTA_ESP32_ERROR_DISCOVERY:
            return APP_OTA_ERROR_DISCOVERY;

        case SVC_OTA_ESP32_ERROR_DOWNLOAD:
            return APP_OTA_ERROR_DOWNLOAD;

        case SVC_OTA_ESP32_ERROR_IMAGE:
            return APP_OTA_ERROR_IMAGE;

        case SVC_OTA_ESP32_ERROR_INIT:
        default:
            return APP_OTA_ERROR_INIT;
    }
}
