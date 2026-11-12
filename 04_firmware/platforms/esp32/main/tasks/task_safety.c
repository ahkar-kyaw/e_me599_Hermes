#include "task_safety.h"

#include "config_control.h"
#include "config_imu.h"
#include "config_rc.h"
#include "safety/safety_balance_control.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "task_control.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_rc.h"
#include "task_supervisor.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TASK_SAFETY_STACK_BYTES 4096u
#define TASK_SAFETY_PRIORITY    (tskIDLE_PRIORITY + 4)

static const char *TAG = "task_safety";

static app_balance_safety_snapshot_t g_balance_snapshot;
static portMUX_TYPE g_balance_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_safety_entry(void *argument);
static void task_safety_publish_balance_snapshot(
    const app_balance_safety_snapshot_t *snapshot);
static void task_safety_build_actuator_command(
    const app_balance_safety_snapshot_t *balance,
    uint64_t now_us,
    app_actuator_command_snapshot_t *command);

void task_safety_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_safety_entry,
        "task_safety",
        TASK_SAFETY_STACK_BYTES,
        NULL,
        TASK_SAFETY_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_safety_get_balance_snapshot(
    app_balance_safety_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_balance_snapshot_lock);
    *snapshot = g_balance_snapshot;
    portEXIT_CRITICAL(&g_balance_snapshot_lock);
}

static void task_safety_entry(void *argument)
{
    (void)argument;

    safety_balance_control_t balance_safety;
    safety_rc_config_t rc_safety_config = safety_rc_default_config();
    safety_imu_config_t imu_safety_config = safety_imu_default_config();
    const safety_balance_control_config_t balance_safety_config =
    {
        .request_max_age_us = APP_BALANCE_REQUEST_MAX_AGE_US,
        .state_max_age_us = APP_BALANCE_STATE_MAX_AGE_US,
        .arm_dwell_us = APP_BALANCE_ARM_DWELL_US,
        .stop_hold_us = APP_BALANCE_STOP_HOLD_US,
        .max_current_a = APP_BALANCE_MAX_CURRENT_A,
        .arm_lean_limit_rad = APP_BALANCE_ARM_LEAN_LIMIT_RAD,
        .arm_lean_rate_limit_rps =
            APP_BALANCE_ARM_LEAN_RATE_LIMIT_RPS,
        .fall_lean_limit_rad = APP_BALANCE_FALL_LEAN_LIMIT_RAD,
    };

    rc_safety_config.max_age_us = APP_RC_MAX_AGE_US;
    rc_safety_config.channel_min = APP_RC_CHANNEL_MIN;
    rc_safety_config.channel_max = APP_RC_CHANNEL_MAX;
    rc_safety_config.channel_count = APP_RC_ACTIVE_CHANNEL_COUNT;
    imu_safety_config.max_age_us = APP_IMU_MAX_AGE_US;
    imu_safety_config.max_abs_roll_rad = APP_IMU_MAX_ABS_ROLL_RAD;
    imu_safety_config.max_abs_pitch_rad = APP_IMU_MAX_ABS_PITCH_RAD;
    imu_safety_config.min_accel_norm_mps2 =
        APP_IMU_MIN_ACCEL_NORM_MPS2;
    imu_safety_config.max_accel_norm_mps2 =
        APP_IMU_MAX_ACCEL_NORM_MPS2;

    if (!safety_balance_control_init(&balance_safety,
                                     &balance_safety_config))
    {
        ESP_LOGE(TAG, "invalid balance safety config");
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        app_balance_safety_request_t request;
        app_balance_safety_snapshot_t balance_snapshot;
        app_actuator_command_snapshot_t actuator_command;
        app_supervisor_snapshot_t supervisor_snapshot;
        actuator_snapshot_t actuator_snapshot;
        control_snapshot_t control_snapshot;
        imu_snapshot_t imu_snapshot;
        rc_snapshot_t rc_snapshot;
        safety_rc_status_t rc_status;
        safety_imu_status_t imu_status;
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        task_supervisor_get_snapshot(&supervisor_snapshot);
        task_rc_get_snapshot(&rc_snapshot);
        task_motor_get_snapshot(&actuator_snapshot);
        task_control_get_snapshot(&control_snapshot);
        task_imu_get_snapshot(&imu_snapshot);

        request.timestamp_us = supervisor_snapshot.timestamp_us;
        request.enabled =
            supervisor_snapshot.mode == APP_SYSTEM_MODE_BALANCE_CONTROL;
        request.motion_neutral =
            (fabsf(supervisor_snapshot.forward_input) <=
             APP_BALANCE_ARM_INPUT_LIMIT) &&
            (fabsf(supervisor_snapshot.lateral_input) <=
             APP_BALANCE_ARM_INPUT_LIMIT) &&
            (fabsf(supervisor_snapshot.yaw_input) <=
             APP_BALANCE_ARM_INPUT_LIMIT);

        safety_rc_check(&rc_snapshot,
                        now_us,
                        &rc_safety_config,
                        &rc_status);
        (void)safety_imu_check(&imu_snapshot,
                               now_us,
                               &imu_safety_config,
                               &imu_status);
        safety_balance_control_update(&balance_safety,
                                      &request,
                                      &control_snapshot,
                                      &rc_snapshot,
                                      &rc_status,
                                      &imu_status,
                                      &actuator_snapshot,
                                      now_us,
                                      &balance_snapshot);

        task_safety_build_actuator_command(&balance_snapshot,
                                           now_us,
                                           &actuator_command);
        task_motor_set_approved_command(&actuator_command);
        task_safety_publish_balance_snapshot(&balance_snapshot);

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(APP_CONTROL_TASK_PERIOD_MS));
    }
}

static void task_safety_publish_balance_snapshot(
    const app_balance_safety_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_balance_snapshot_lock);
    g_balance_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_balance_snapshot_lock);
}

static void task_safety_build_actuator_command(
    const app_balance_safety_snapshot_t *balance,
    uint64_t now_us,
    app_actuator_command_snapshot_t *command)
{
    memset(command, 0, sizeof(*command));
    command->timestamp_us = now_us;
    command->valid = true;

    if (!balance->transmit_enabled)
    {
        return;
    }

    for (uint32_t actuator = 0u;
         actuator < APP_ACTUATOR_COUNT;
         actuator++)
    {
        command->actuator[actuator].enabled = true;
        command->actuator[actuator].current_a =
            balance->approved_current_a[actuator];
    }
}
