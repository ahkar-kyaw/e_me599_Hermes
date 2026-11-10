#include "task_safety.h"

#include "config_actuator.h"
#include "config_control.h"
#include "config_imu.h"
#include "config_rc.h"
#include "safety/safety_manual_drive.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "safety/safety_torque_control.h"
#include "task_control.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_rc.h"
#include "task_supervisor.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <string.h>

#define TASK_SAFETY_STACK_BYTES 4096u
#define TASK_SAFETY_PRIORITY    (tskIDLE_PRIORITY + 4)

static const char *TAG = "task_safety";

static app_manual_drive_snapshot_t g_manual_drive_snapshot;
static portMUX_TYPE g_manual_drive_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;
static app_torque_control_snapshot_t g_torque_control_snapshot;
static portMUX_TYPE g_torque_control_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_safety_entry(void *argument);
static void task_safety_publish_manual_drive_snapshot(
    const app_manual_drive_snapshot_t *snapshot);
static void task_safety_publish_torque_control_snapshot(
    const app_torque_control_snapshot_t *snapshot);
static void task_safety_build_actuator_command(
    const app_manual_drive_snapshot_t *manual_drive,
    const app_torque_control_snapshot_t *torque_control,
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

void task_safety_get_manual_drive_snapshot(
    app_manual_drive_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_manual_drive_snapshot_lock);
    *snapshot = g_manual_drive_snapshot;
    portEXIT_CRITICAL(&g_manual_drive_snapshot_lock);
}

void task_safety_get_torque_control_snapshot(
    app_torque_control_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_torque_control_snapshot_lock);
    *snapshot = g_torque_control_snapshot;
    portEXIT_CRITICAL(&g_torque_control_snapshot_lock);
}

static void task_safety_entry(void *argument)
{
    (void)argument;

    safety_manual_drive_t manual_drive;
    safety_torque_control_t torque_control;
    safety_rc_config_t rc_safety_config = safety_rc_default_config();
    safety_imu_config_t imu_safety_config = safety_imu_default_config();
    const safety_manual_drive_config_t manual_drive_config =
    {
        .request_max_age_us = APP_MANUAL_DRIVE_REQUEST_MAX_AGE_US,
        .stop_hold_us = APP_MANUAL_DRIVE_STOP_HOLD_US,
        .channel_min = APP_RC_CHANNEL_MIN,
        .channel_max = APP_RC_CHANNEL_MAX,
        .stick_neutral_low = APP_MANUAL_DRIVE_STICK_NEUTRAL_LOW,
        .stick_neutral_high = APP_MANUAL_DRIVE_STICK_NEUTRAL_HIGH,
        .max_velocity_erpm = APP_MANUAL_DRIVE_MAX_VELOCITY_ERPM,
        .velocity_channel =
            (uint8_t)(APP_MANUAL_DRIVE_VELOCITY_CHANNEL - 1u),
        .velocity_positive_high =
            (APP_MANUAL_DRIVE_VELOCITY_POSITIVE_HIGH != 0),
    };
    const safety_torque_control_config_t torque_control_config =
    {
        .request_max_age_us = APP_CONTROL_REQUEST_MAX_AGE_US,
        .state_max_age_us = APP_CONTROL_STATE_MAX_AGE_US,
        .stop_hold_us = APP_CONTROL_STOP_HOLD_US,
        .stick_neutral_low = APP_MANUAL_DRIVE_STICK_NEUTRAL_LOW,
        .stick_neutral_high = APP_MANUAL_DRIVE_STICK_NEUTRAL_HIGH,
        .max_current_a = APP_CONTROL_TORQUE_MAX_CURRENT_A,
        .arm_lean_limit_rad = APP_CONTROL_ARM_LEAN_LIMIT_RAD,
        .arm_lean_rate_limit_rps =
            APP_CONTROL_ARM_LEAN_RATE_LIMIT_RPS,
        .fall_lean_limit_rad = APP_CONTROL_FALL_LEAN_LIMIT_RAD,
        .current_channel =
            (uint8_t)(APP_CONTROL_TORQUE_INPUT_CHANNEL - 1u),
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

    if (!safety_manual_drive_init(&manual_drive,
                                  &manual_drive_config))
    {
        ESP_LOGE(TAG, "invalid manual-drive safety config");
        vTaskDelete(NULL);
        return;
    }

    if (!safety_torque_control_init(&torque_control,
                                    &torque_control_config))
    {
        ESP_LOGE(TAG, "invalid torque-control safety config");
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        app_manual_drive_request_t request;
        app_torque_control_request_t torque_request;
        app_manual_drive_snapshot_t drive_snapshot;
        app_torque_control_snapshot_t torque_snapshot;
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
        request.actuator_index =
            supervisor_snapshot.manual_drive_actuator_index;
        request.enabled =
            supervisor_snapshot.mode == APP_SYSTEM_MODE_MANUAL_DRIVE;

        torque_request.timestamp_us = supervisor_snapshot.timestamp_us;
        torque_request.enabled =
            (supervisor_snapshot.mode == APP_SYSTEM_MODE_TORQUE_CONTROL) ||
            (supervisor_snapshot.mode == APP_SYSTEM_MODE_BALANCE_CONTROL);
        torque_request.stick_must_remain_neutral =
            supervisor_snapshot.mode == APP_SYSTEM_MODE_BALANCE_CONTROL;

        safety_rc_check(&rc_snapshot,
                        now_us,
                        &rc_safety_config,
                        &rc_status);
        (void)safety_imu_check(&imu_snapshot,
                               now_us,
                               &imu_safety_config,
                               &imu_status);
        safety_manual_drive_update(&manual_drive,
                                   &request,
                                   &rc_snapshot,
                                   &rc_status,
                                   &actuator_snapshot,
                                   now_us,
                                   &drive_snapshot);

        safety_torque_control_update(&torque_control,
                                     &torque_request,
                                     &control_snapshot,
                                     &rc_snapshot,
                                     &rc_status,
                                     &imu_status,
                                     &actuator_snapshot,
                                     now_us,
                                     &torque_snapshot);

        task_safety_build_actuator_command(&drive_snapshot,
                                           &torque_snapshot,
                                           now_us,
                                           &actuator_command);
        task_motor_set_approved_command(&actuator_command);
        task_safety_publish_manual_drive_snapshot(&drive_snapshot);
        task_safety_publish_torque_control_snapshot(&torque_snapshot);

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(APP_CONTROL_TASK_PERIOD_MS));
    }
}

static void task_safety_publish_manual_drive_snapshot(
    const app_manual_drive_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_manual_drive_snapshot_lock);
    g_manual_drive_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_manual_drive_snapshot_lock);
}

static void task_safety_publish_torque_control_snapshot(
    const app_torque_control_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_torque_control_snapshot_lock);
    g_torque_control_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_torque_control_snapshot_lock);
}

static void task_safety_build_actuator_command(
    const app_manual_drive_snapshot_t *manual_drive,
    const app_torque_control_snapshot_t *torque_control,
    uint64_t now_us,
    app_actuator_command_snapshot_t *command)
{
    memset(command, 0, sizeof(*command));
    command->timestamp_us = now_us;
    command->valid = true;

    if (torque_control->transmit_enabled)
    {
        for (uint32_t actuator = 0u;
             actuator < APP_ACTUATOR_COUNT;
             actuator++)
        {
            command->actuator[actuator].enabled = true;
            command->actuator[actuator].mode =
                APP_ACTUATOR_COMMAND_SERVO_CURRENT;
            command->actuator[actuator].value =
                torque_control->approved_current_a[actuator];
        }

        return;
    }

    if (manual_drive->transmit_enabled &&
        (manual_drive->actuator_index < APP_ACTUATOR_COUNT))
    {
        app_actuator_command_entry_t *entry =
            &command->actuator[manual_drive->actuator_index];

        entry->enabled = true;
        entry->mode = APP_ACTUATOR_COMMAND_SERVO_VELOCITY;
        entry->value = manual_drive->velocity_erpm;
    }
}
