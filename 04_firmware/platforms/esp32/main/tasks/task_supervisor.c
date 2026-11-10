#include "task_supervisor.h"

#include "config_actuator.h"
#include "config_control.h"
#include "task_ui.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <string.h>

#define TASK_SUPERVISOR_STACK_BYTES 3072u
#define TASK_SUPERVISOR_PRIORITY    (tskIDLE_PRIORITY + 2)

static const char *TAG = "task_supervisor";

static app_supervisor_snapshot_t g_supervisor_snapshot;
static portMUX_TYPE g_supervisor_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_supervisor_entry(void *argument);
static void task_supervisor_publish_snapshot(
    const app_supervisor_snapshot_t *snapshot);
static bool task_supervisor_request_fresh(
    const app_manual_drive_request_t *request,
    uint64_t now_us);
static bool task_supervisor_torque_request_fresh(
    const app_torque_control_request_t *request,
    uint64_t now_us);
static bool task_supervisor_balance_request_fresh(
    const app_balance_control_request_t *request,
    uint64_t now_us);

void task_supervisor_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_supervisor_entry,
        "task_supervisor",
        TASK_SUPERVISOR_STACK_BYTES,
        NULL,
        TASK_SUPERVISOR_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_supervisor_get_snapshot(
    app_supervisor_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_supervisor_snapshot_lock);
    *snapshot = g_supervisor_snapshot;
    portEXIT_CRITICAL(&g_supervisor_snapshot_lock);
}

static void task_supervisor_entry(void *argument)
{
    (void)argument;

    app_supervisor_snapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.mode = APP_SYSTEM_MODE_SAFE_IDLE;
    task_supervisor_publish_snapshot(&snapshot);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        app_manual_drive_request_t request;
        app_torque_control_request_t torque_request;
        app_balance_control_request_t balance_request;
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        task_ui_get_manual_drive_request(&request);
        task_ui_get_torque_control_request(&torque_request);
        task_ui_get_balance_control_request(&balance_request);

        snapshot.timestamp_us = now_us;
        snapshot.mode = APP_SYSTEM_MODE_SAFE_IDLE;
        snapshot.manual_drive_actuator_index = 0u;

        if (task_supervisor_balance_request_fresh(&balance_request,
                                                  now_us) &&
            balance_request.enabled)
        {
            snapshot.mode = APP_SYSTEM_MODE_BALANCE_CONTROL;
        }
        else if (task_supervisor_torque_request_fresh(&torque_request,
                                                      now_us) &&
            torque_request.enabled)
        {
            snapshot.mode = APP_SYSTEM_MODE_TORQUE_CONTROL;
        }
        else if (task_supervisor_request_fresh(&request, now_us) &&
            request.enabled &&
            (request.actuator_index < APP_ACTUATOR_COUNT))
        {
            snapshot.mode = APP_SYSTEM_MODE_MANUAL_DRIVE;
            snapshot.manual_drive_actuator_index =
                request.actuator_index;
        }

        task_supervisor_publish_snapshot(&snapshot);

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(APP_MANUAL_DRIVE_TASK_PERIOD_MS));
    }
}

static void task_supervisor_publish_snapshot(
    const app_supervisor_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_supervisor_snapshot_lock);
    g_supervisor_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_supervisor_snapshot_lock);
}

static bool task_supervisor_request_fresh(
    const app_manual_drive_request_t *request,
    uint64_t now_us)
{
    return (request->timestamp_us != 0u) &&
           (now_us >= request->timestamp_us) &&
           ((now_us - request->timestamp_us) <=
            APP_MANUAL_DRIVE_REQUEST_MAX_AGE_US);
}

static bool task_supervisor_torque_request_fresh(
    const app_torque_control_request_t *request,
    uint64_t now_us)
{
    return (request->timestamp_us != 0u) &&
           (now_us >= request->timestamp_us) &&
           ((now_us - request->timestamp_us) <=
            APP_CONTROL_REQUEST_MAX_AGE_US);
}

static bool task_supervisor_balance_request_fresh(
    const app_balance_control_request_t *request,
    uint64_t now_us)
{
    return (request->timestamp_us != 0u) &&
           (now_us >= request->timestamp_us) &&
           ((now_us - request->timestamp_us) <=
            APP_CONTROL_REQUEST_MAX_AGE_US);
}
