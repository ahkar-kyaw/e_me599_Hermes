#include "task_control.h"

#include "app/app_balance_state.h"
#include "config_actuator.h"
#include "config_control.h"
#include "config_imu.h"
#include "config_rc.h"
#include "control/ctrl_balance_controller.h"
#include "control/ctrl_torque_input.h"
#include "estimation/est_collinear_mecanum.h"
#include "safety/safety_imu.h"
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

#define TASK_CONTROL_STACK_BYTES 4096u
#define TASK_CONTROL_PRIORITY    (tskIDLE_PRIORITY + 4)

_Static_assert(APP_ACTUATOR_COUNT == EST_COLLINEAR_MECANUM_WHEEL_COUNT,
               "control estimator requires four actuator slots");
_Static_assert(APP_ACTUATOR_COUNT == CTRL_BALANCE_ACTUATOR_COUNT,
               "control output requires four actuator slots");
_Static_assert(configTICK_RATE_HZ >= 200,
               "control task requires an RTOS tick rate of at least 200 Hz");
_Static_assert((APP_CONTROL_TORQUE_INPUT_CHANNEL >= 1u) &&
               (APP_CONTROL_TORQUE_INPUT_CHANNEL <= APP_RC_CHANNEL_COUNT),
               "torque input channel is out of range");

static const char *TAG = "task_control";

static control_snapshot_t g_control_snapshot;
static portMUX_TYPE g_control_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_control_entry(void *argument);
static void task_control_publish_snapshot(
    const control_snapshot_t *snapshot);
static uint32_t task_control_actuator_faults(
    const actuator_snapshot_t *actuator_snapshot);

void task_control_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_control_entry,
        "task_control",
        TASK_CONTROL_STACK_BYTES,
        NULL,
        TASK_CONTROL_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_control_get_snapshot(control_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_control_snapshot_lock);
    *snapshot = g_control_snapshot;
    portEXIT_CRITICAL(&g_control_snapshot_lock);
}

static void task_control_entry(void *argument)
{
    (void)argument;

    est_collinear_mecanum_t odometry_estimator;
    const est_collinear_mecanum_config_t odometry_config =
    {
        .wheel_radius_m = APP_CONTROL_WHEEL_RADIUS_M,
        .wheel_lateral_m =
        {
            APP_CONTROL_WHEEL_1_LATERAL_M,
            APP_CONTROL_WHEEL_2_LATERAL_M,
            APP_CONTROL_WHEEL_3_LATERAL_M,
            APP_CONTROL_WHEEL_4_LATERAL_M,
        },
        .roller_lateral_factor =
        {
            APP_CONTROL_WHEEL_1_ROLLER_FACTOR,
            APP_CONTROL_WHEEL_2_ROLLER_FACTOR,
            APP_CONTROL_WHEEL_3_ROLLER_FACTOR,
            APP_CONTROL_WHEEL_4_ROLLER_FACTOR,
        },
        .max_dt_s = APP_CONTROL_MAX_DT_S,
    };
    safety_imu_config_t imu_safety_config =
        safety_imu_default_config();
    ctrl_balance_controller_t balance_controller;
    const ctrl_balance_controller_config_t balance_controller_config =
    {
        .lean_trim_rad = APP_BALANCE_LEAN_TRIM_RAD,
        .lean_gain_a_per_rad = APP_BALANCE_LEAN_GAIN_A_PER_RAD,
        .lean_rate_gain_a_per_rps =
            APP_BALANCE_LEAN_RATE_GAIN_A_PER_RPS,
        .position_gain_a_per_m = APP_BALANCE_POSITION_GAIN_A_PER_M,
        .velocity_gain_a_per_mps =
            APP_BALANCE_VELOCITY_GAIN_A_PER_MPS,
        .position_integral_gain_a_per_m_s =
            APP_BALANCE_POSITION_INTEGRAL_GAIN_A_PER_M_S,
        .position_integral_limit_m_s =
            APP_BALANCE_POSITION_INTEGRAL_LIMIT_M_S,
        .max_current_a = APP_CONTROL_TORQUE_MAX_CURRENT_A,
        .output_sign = APP_BALANCE_OUTPUT_SIGN,
    };
    ctrl_torque_input_t torque_input;
    const ctrl_torque_input_config_t torque_input_config =
    {
        .channel_min = APP_RC_CHANNEL_MIN,
        .channel_max = APP_RC_CHANNEL_MAX,
        .neutral_low = APP_MANUAL_DRIVE_STICK_NEUTRAL_LOW,
        .neutral_high = APP_MANUAL_DRIVE_STICK_NEUTRAL_HIGH,
        .max_current_a = APP_CONTROL_TORQUE_MAX_CURRENT_A,
        .slew_rate_a_per_s = APP_CONTROL_TORQUE_SLEW_RATE_A_PER_S,
        .positive_high =
            (APP_CONTROL_TORQUE_INPUT_POSITIVE_HIGH != 0),
    };
    control_snapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    task_control_publish_snapshot(&snapshot);

    imu_safety_config.max_age_us = APP_IMU_MAX_AGE_US;
    imu_safety_config.max_abs_roll_rad = APP_IMU_MAX_ABS_ROLL_RAD;
    imu_safety_config.max_abs_pitch_rad = APP_IMU_MAX_ABS_PITCH_RAD;
    imu_safety_config.min_accel_norm_mps2 =
        APP_IMU_MIN_ACCEL_NORM_MPS2;
    imu_safety_config.max_accel_norm_mps2 =
        APP_IMU_MAX_ACCEL_NORM_MPS2;

    if (!est_collinear_mecanum_init(&odometry_estimator,
                                     &odometry_config))
    {
        ESP_LOGE(TAG, "invalid odometry configuration");
        vTaskDelete(NULL);
        return;
    }

    if (!ctrl_torque_input_init(&torque_input,
                                &torque_input_config))
    {
        ESP_LOGE(TAG, "invalid torque input configuration");
        vTaskDelete(NULL);
        return;
    }

    if (!ctrl_balance_controller_init(&balance_controller,
                                      &balance_controller_config))
    {
        ESP_LOGE(TAG, "invalid balance controller configuration");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "200 Hz state estimator and controller started");

    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_loop_us = 0u;
    bool balance_reference_captured = false;

    for (;;)
    {
        const uint64_t loop_start_us = (uint64_t)esp_timer_get_time();
        const float dt_s = (last_loop_us == 0u) ?
            ((float)APP_CONTROL_TASK_PERIOD_MS * 0.001f) :
            ((float)(loop_start_us - last_loop_us) * 0.000001f);
        imu_snapshot_t imu_snapshot;
        actuator_snapshot_t actuator_snapshot;
        app_supervisor_snapshot_t supervisor_snapshot;
        rc_snapshot_t rc_snapshot;
        safety_imu_status_t imu_status;
        est_collinear_mecanum_output_t odometry;

        last_loop_us = loop_start_us;
        snapshot.timestamp_us = loop_start_us;
        snapshot.loop_count++;
        snapshot.input_fault_flags = CTRL_STATE_FAULT_NONE;
        memset(&snapshot.state, 0, sizeof(snapshot.state));
        memset(snapshot.desired_current_a,
               0,
               sizeof(snapshot.desired_current_a));
        snapshot.state.timestamp_us = loop_start_us;
        snapshot.requested_current_a = 0.0f;
        snapshot.balance_reference_position_x_m = 0.0f;
        snapshot.balance_position_error_m = 0.0f;
        snapshot.balance_position_integral_m_s = 0.0f;
        snapshot.balance_lean_error_rad = 0.0f;
        snapshot.balance_common_current_a = 0.0f;
        snapshot.command_mode = CTRL_COMMAND_MODE_NONE;
        snapshot.torque_command_valid = false;
        snapshot.balance_output_saturated = false;

        task_imu_get_snapshot(&imu_snapshot);
        task_motor_get_snapshot(&actuator_snapshot);
        task_rc_get_snapshot(&rc_snapshot);
        task_supervisor_get_snapshot(&supervisor_snapshot);
        (void)safety_imu_check(&imu_snapshot,
                               loop_start_us,
                               &imu_safety_config,
                               &imu_status);

        snapshot.input_fault_flags = task_control_actuator_faults(
            &actuator_snapshot);

        if (!app_balance_state_imu_ready(&imu_status))
        {
            snapshot.input_fault_flags |= CTRL_STATE_FAULT_IMU;
        }

        if ((dt_s <= 0.0f) || (dt_s > APP_CONTROL_MAX_DT_S))
        {
            snapshot.input_fault_flags |= CTRL_STATE_FAULT_TIMING;
        }

        bool odometry_updated = false;

        if ((snapshot.input_fault_flags &
             (CTRL_STATE_FAULT_ACTUATOR | CTRL_STATE_FAULT_TIMING)) == 0u)
        {
            float wheel_velocity_radps[APP_ACTUATOR_COUNT];

            for (uint32_t wheel = 0u;
                 wheel < APP_ACTUATOR_COUNT;
                 wheel++)
            {
                wheel_velocity_radps[wheel] =
                    actuator_snapshot.actuator[wheel].velocity_erpm *
                    APP_CONTROL_ERPM_TO_WHEEL_RADPS;
            }

            odometry_updated = est_collinear_mecanum_update(
                &odometry_estimator,
                wheel_velocity_radps,
                dt_s,
                &odometry);

            if (!odometry_updated)
            {
                snapshot.input_fault_flags |=
                    CTRL_STATE_FAULT_ESTIMATOR;
            }
        }

        if (odometry_updated &&
            ((snapshot.input_fault_flags & CTRL_STATE_FAULT_IMU) == 0u) &&
            !app_balance_state_compose(
                &imu_snapshot,
                &imu_status,
                &odometry,
                APP_CONTROL_LEAN_AXIS,
                loop_start_us,
                &snapshot.state))
        {
            snapshot.input_fault_flags |= CTRL_STATE_FAULT_ESTIMATOR;
        }

        if (supervisor_snapshot.mode == APP_SYSTEM_MODE_TORQUE_CONTROL)
        {
            balance_reference_captured = false;
            const uint8_t channel =
                (uint8_t)(APP_CONTROL_TORQUE_INPUT_CHANNEL - 1u);
            float requested_current_a = 0.0f;

            if (snapshot.state.valid && rc_snapshot.valid &&
                ctrl_torque_input_update(
                    &torque_input,
                    rc_snapshot.channel[channel],
                    dt_s,
                    &requested_current_a))
            {
                static const float torque_mix[APP_ACTUATOR_COUNT] =
                {
                    APP_CONTROL_TORQUE_MIX_1,
                    APP_CONTROL_TORQUE_MIX_2,
                    APP_CONTROL_TORQUE_MIX_3,
                    APP_CONTROL_TORQUE_MIX_4,
                };

                snapshot.requested_current_a = requested_current_a;
                snapshot.command_mode =
                    CTRL_COMMAND_MODE_MANUAL_CURRENT;

                for (uint32_t actuator = 0u;
                     actuator < APP_ACTUATOR_COUNT;
                     actuator++)
                {
                    snapshot.desired_current_a[actuator] =
                        requested_current_a * torque_mix[actuator];
                }

                snapshot.torque_command_valid = true;
            }
        }
        else if (supervisor_snapshot.mode ==
                 APP_SYSTEM_MODE_BALANCE_CONTROL)
        {
            static const float balance_mix[APP_ACTUATOR_COUNT] =
            {
                APP_BALANCE_MIX_1,
                APP_BALANCE_MIX_2,
                APP_BALANCE_MIX_3,
                APP_BALANCE_MIX_4,
            };
            ctrl_balance_controller_output_t balance_output;

            ctrl_torque_input_reset(&torque_input);

            if (snapshot.state.valid && !balance_reference_captured)
            {
                ctrl_balance_controller_reset(
                    &balance_controller,
                    snapshot.state.position_x_m);
                balance_reference_captured = true;
            }

            if (balance_reference_captured &&
                ctrl_balance_controller_update(
                    &balance_controller,
                    &snapshot.state,
                    dt_s,
                    &balance_output))
            {
                snapshot.balance_reference_position_x_m =
                    balance_output.reference_position_x_m;
                snapshot.balance_position_error_m =
                    balance_output.position_error_m;
                snapshot.balance_position_integral_m_s =
                    balance_output.position_integral_m_s;
                snapshot.balance_lean_error_rad =
                    balance_output.lean_error_rad;
                snapshot.balance_common_current_a =
                    balance_output.common_current_a;
                snapshot.balance_output_saturated =
                    balance_output.saturated;
                snapshot.command_mode = CTRL_COMMAND_MODE_BALANCE;

                for (uint32_t actuator = 0u;
                     actuator < APP_ACTUATOR_COUNT;
                     actuator++)
                {
                    snapshot.desired_current_a[actuator] =
                        balance_output.common_current_a *
                        balance_mix[actuator];
                }

                snapshot.torque_command_valid = true;
            }
        }
        else
        {
            ctrl_torque_input_reset(&torque_input);
            balance_reference_captured = false;
        }

        const uint64_t loop_end_us = (uint64_t)esp_timer_get_time();
        const uint64_t execution_time_us = loop_end_us - loop_start_us;
        snapshot.execution_time_us = (execution_time_us > UINT32_MAX) ?
            UINT32_MAX : (uint32_t)execution_time_us;

        if (snapshot.execution_time_us > snapshot.max_execution_time_us)
        {
            snapshot.max_execution_time_us = snapshot.execution_time_us;
        }

        if (execution_time_us >=
            ((uint64_t)APP_CONTROL_TASK_PERIOD_MS * 1000u))
        {
            snapshot.overrun_count++;
            snapshot.input_fault_flags |= CTRL_STATE_FAULT_TIMING;
            snapshot.state.valid = false;
        }

        if (!snapshot.state.valid)
        {
            snapshot.invalid_input_count++;
        }

        task_control_publish_snapshot(&snapshot);

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_CONTROL_TASK_PERIOD_MS));
    }
}

static void task_control_publish_snapshot(
    const control_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_control_snapshot_lock);
    g_control_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_control_snapshot_lock);
}

static uint32_t task_control_actuator_faults(
    const actuator_snapshot_t *actuator_snapshot)
{
    uint32_t fault_flags = CTRL_STATE_FAULT_NONE;

    if (!actuator_snapshot->bus_initialized ||
        (actuator_snapshot->bus_state != APP_ACTUATOR_BUS_ACTIVE))
    {
        fault_flags |= CTRL_STATE_FAULT_ACTUATOR;
    }

    for (uint32_t wheel = 0u; wheel < APP_ACTUATOR_COUNT; wheel++)
    {
        const actuator_feedback_snapshot_t *feedback =
            &actuator_snapshot->actuator[wheel];

        if (!feedback->configured || !feedback->valid ||
            (feedback->fault_code != 0u))
        {
            fault_flags |= CTRL_STATE_FAULT_ACTUATOR;
        }
    }

    return fault_flags;
}
