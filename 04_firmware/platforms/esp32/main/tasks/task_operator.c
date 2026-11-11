#include "task_operator.h"

#include "config_rc.h"
#include "safety/safety_rc.h"
#include "task_rc.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <string.h>

#define TASK_OPERATOR_STACK_BYTES 3072u
#define TASK_OPERATOR_PRIORITY    (tskIDLE_PRIORITY + 2)

_Static_assert((APP_RC_MASTER_ENABLE_CHANNEL >= 1u) &&
               (APP_RC_MASTER_ENABLE_CHANNEL <=
                APP_RC_ACTIVE_CHANNEL_COUNT),
               "master channel is outside the active RC range");
_Static_assert((APP_RC_BALANCE_ENABLE_CHANNEL >= 1u) &&
               (APP_RC_BALANCE_ENABLE_CHANNEL <=
                APP_RC_ACTIVE_CHANNEL_COUNT),
               "balance channel is outside the active RC range");
_Static_assert((APP_RC_ZERO_LEAN_CHANNEL >= 1u) &&
               (APP_RC_ZERO_LEAN_CHANNEL <=
                APP_RC_ACTIVE_CHANNEL_COUNT),
               "zero channel is outside the active RC range");
_Static_assert((APP_RC_FORWARD_CHANNEL >= 1u) &&
               (APP_RC_FORWARD_CHANNEL <= APP_RC_ACTIVE_CHANNEL_COUNT) &&
               (APP_RC_LATERAL_CHANNEL >= 1u) &&
               (APP_RC_LATERAL_CHANNEL <= APP_RC_ACTIVE_CHANNEL_COUNT) &&
               (APP_RC_YAW_CHANNEL >= 1u) &&
               (APP_RC_YAW_CHANNEL <= APP_RC_ACTIVE_CHANNEL_COUNT),
               "motion channel is outside the active RC range");

typedef struct
{
    app_balance_control_request_t balance_request;
    app_operator_controls_state_t controls;
} task_operator_snapshot_t;

static const char *TAG = "task_operator";

static task_operator_snapshot_t g_operator_snapshot;
static portMUX_TYPE g_operator_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_operator_entry(void *argument);
static void task_operator_publish_snapshot(
    const task_operator_snapshot_t *snapshot);

void task_operator_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_operator_entry,
        "task_operator",
        TASK_OPERATOR_STACK_BYTES,
        NULL,
        TASK_OPERATOR_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_operator_get_balance_control_request(
    app_balance_control_request_t *request)
{
    if (request == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_operator_snapshot_lock);
    *request = g_operator_snapshot.balance_request;
    portEXIT_CRITICAL(&g_operator_snapshot_lock);
}

void task_operator_get_controls_state(
    app_operator_controls_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_operator_snapshot_lock);
    *state = g_operator_snapshot.controls;
    portEXIT_CRITICAL(&g_operator_snapshot_lock);
}

static void task_operator_entry(void *argument)
{
    (void)argument;

    const app_operator_controls_config_t controls_config =
    {
        .master_channel =
            (uint8_t)(APP_RC_MASTER_ENABLE_CHANNEL - 1u),
        .balance_channel =
            (uint8_t)(APP_RC_BALANCE_ENABLE_CHANNEL - 1u),
        .zero_lean_channel =
            (uint8_t)(APP_RC_ZERO_LEAN_CHANNEL - 1u),
        .forward_channel =
            (uint8_t)(APP_RC_FORWARD_CHANNEL - 1u),
        .lateral_channel =
            (uint8_t)(APP_RC_LATERAL_CHANNEL - 1u),
        .yaw_channel =
            (uint8_t)(APP_RC_YAW_CHANNEL - 1u),
        .channel_min = APP_RC_CHANNEL_MIN,
        .channel_max = APP_RC_CHANNEL_MAX,
        .input_min = APP_RC_INPUT_MIN,
        .input_mid = APP_RC_INPUT_MID,
        .input_max = APP_RC_INPUT_MAX,
        .input_deadband = APP_RC_INPUT_DEADBAND,
        .switch_off_threshold = APP_RC_SWITCH_OFF_THRESHOLD,
        .switch_on_threshold = APP_RC_SWITCH_ON_THRESHOLD,
        .master_active_high = (APP_RC_MASTER_ACTIVE_HIGH != 0),
        .balance_active_high = (APP_RC_BALANCE_ACTIVE_HIGH != 0),
        .zero_lean_active_high = (APP_RC_ZERO_ACTIVE_HIGH != 0),
        .forward_inverted = (APP_RC_FORWARD_INVERTED != 0),
        .lateral_inverted = (APP_RC_LATERAL_INVERTED != 0),
        .yaw_inverted = (APP_RC_YAW_INVERTED != 0),
    };
    app_operator_controls_t controls;
    safety_rc_config_t rc_safety_config = safety_rc_default_config();
    task_operator_snapshot_t snapshot;
    uint32_t zero_lean_request_count = 0u;

    memset(&snapshot, 0, sizeof(snapshot));
    task_operator_publish_snapshot(&snapshot);

    rc_safety_config.max_age_us = APP_RC_MAX_AGE_US;
    rc_safety_config.channel_min = APP_RC_CHANNEL_MIN;
    rc_safety_config.channel_max = APP_RC_CHANNEL_MAX;
    rc_safety_config.channel_count = APP_RC_ACTIVE_CHANNEL_COUNT;

    if (!app_operator_controls_init(&controls, &controls_config))
    {
        ESP_LOGE(TAG, "invalid operator control config");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "operator controls ready: SA=CH%u SD=CH%u zero=CH%u",
             (unsigned int)APP_RC_MASTER_ENABLE_CHANNEL,
             (unsigned int)APP_RC_BALANCE_ENABLE_CHANNEL,
             (unsigned int)APP_RC_ZERO_LEAN_CHANNEL);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        rc_snapshot_t rc_snapshot;
        safety_rc_status_t rc_status;
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        task_rc_get_snapshot(&rc_snapshot);
        safety_rc_check(&rc_snapshot,
                        now_us,
                        &rc_safety_config,
                        &rc_status);
        app_operator_controls_update(&controls,
                                     &rc_snapshot,
                                     rc_status.safe_for_control,
                                     &snapshot.controls);

        if (snapshot.controls.zero_lean_requested &&
            snapshot.controls.master_enabled &&
            !snapshot.controls.balance_enabled)
        {
            zero_lean_request_count++;
        }

        snapshot.balance_request.timestamp_us = now_us;
        snapshot.balance_request.zero_lean_request_count =
            zero_lean_request_count;
        snapshot.balance_request.forward_input =
            snapshot.controls.forward_input;
        snapshot.balance_request.lateral_input =
            snapshot.controls.lateral_input;
        snapshot.balance_request.yaw_input =
            snapshot.controls.yaw_input;
        snapshot.balance_request.master_enabled =
            snapshot.controls.link_usable &&
            snapshot.controls.master_enabled;
        snapshot.balance_request.balance_enabled =
            snapshot.controls.link_usable &&
            snapshot.controls.balance_enabled;

        task_operator_publish_snapshot(&snapshot);

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_OPERATOR_TASK_PERIOD_MS));
    }
}

static void task_operator_publish_snapshot(
    const task_operator_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_operator_snapshot_lock);
    g_operator_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_operator_snapshot_lock);
}
