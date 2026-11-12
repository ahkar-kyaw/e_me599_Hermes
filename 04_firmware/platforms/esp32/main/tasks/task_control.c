#include "task_control.h"

#include "app/app_balance_state.h"
#include "config_actuator.h"
#include "config_control.h"
#include "config_imu.h"
#include "control/ctrl_balance_controller.h"
#include "control/ctrl_collinear_mecanum_allocator.h"
#include "estimation/est_collinear_mecanum.h"
#include "safety/safety_imu.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_safety.h"
#include "task_supervisor.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TASK_CONTROL_STACK_BYTES 4096u
#define TASK_CONTROL_PRIORITY    (tskIDLE_PRIORITY + 4)

_Static_assert(APP_ACTUATOR_COUNT == EST_COLLINEAR_MECANUM_WHEEL_COUNT,
               "control estimator requires four actuator slots");
_Static_assert(APP_ACTUATOR_COUNT == CTRL_BALANCE_ACTUATOR_COUNT,
               "control output requires four actuator slots");
_Static_assert(APP_ACTUATOR_COUNT == CTRL_COLLINEAR_MECANUM_WHEEL_COUNT,
               "control allocator requires four actuator slots");
_Static_assert(configTICK_RATE_HZ >= 200,
               "control task requires an RTOS tick rate of at least 200 Hz");

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
        .wheel_offset_m =
        {
            APP_CONTROL_WHEEL_1_OFFSET_M,
            APP_CONTROL_WHEEL_2_OFFSET_M,
            APP_CONTROL_WHEEL_3_OFFSET_M,
            APP_CONTROL_WHEEL_4_OFFSET_M,
        },
        .roller_angle_rad =
        {
            APP_CONTROL_WHEEL_1_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_2_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_3_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_4_ROLLER_ANGLE_RAD,
        },
        .max_dt_s = APP_CONTROL_MAX_DT_S,
    };
    safety_imu_config_t imu_safety_config =
        safety_imu_default_config();
    ctrl_balance_controller_t balance_controller;
    ctrl_collinear_mecanum_allocator_t current_allocator;
    const ctrl_balance_controller_config_t balance_controller_config =
    {
        .lean_gain_a_per_rad = APP_BALANCE_LEAN_GAIN_A_PER_RAD,
        .lean_rate_gain_a_per_rps =
            APP_BALANCE_LEAN_RATE_GAIN_A_PER_RPS,
        .forward_position_gain_a_per_m =
            APP_BALANCE_FORWARD_POSITION_GAIN_A_PER_M,
        .forward_velocity_gain_a_per_mps =
            APP_BALANCE_FORWARD_VELOCITY_GAIN_A_PER_MPS,
        .forward_integral_gain_a_per_m_s =
            APP_BALANCE_FORWARD_INTEGRAL_GAIN_A_PER_M_S,
        .forward_integral_limit_m_s =
            APP_BALANCE_FORWARD_INTEGRAL_LIMIT_M_S,
        .lateral_position_gain_a_per_m =
            APP_BALANCE_LATERAL_POSITION_GAIN_A_PER_M,
        .lateral_velocity_gain_a_per_mps =
            APP_BALANCE_LATERAL_VELOCITY_GAIN_A_PER_MPS,
        .lateral_integral_gain_a_per_m_s =
            APP_BALANCE_LATERAL_INTEGRAL_GAIN_A_PER_M_S,
        .lateral_integral_limit_m_s =
            APP_BALANCE_LATERAL_INTEGRAL_LIMIT_M_S,
        .yaw_position_gain_a_per_rad =
            APP_BALANCE_YAW_POSITION_GAIN_A_PER_RAD,
        .yaw_rate_gain_a_per_rps =
            APP_BALANCE_YAW_RATE_GAIN_A_PER_RPS,
        .yaw_integral_gain_a_per_rad_s =
            APP_BALANCE_YAW_INTEGRAL_GAIN_A_PER_RAD_S,
        .yaw_integral_limit_rad_s =
            APP_BALANCE_YAW_INTEGRAL_LIMIT_RAD_S,
        .max_forward_accel_mps2 =
            APP_MOTION_MAX_FORWARD_ACCEL_MPS2,
        .max_lateral_accel_mps2 =
            APP_MOTION_MAX_LATERAL_ACCEL_MPS2,
        .max_yaw_accel_rps2 = APP_MOTION_MAX_YAW_ACCEL_RPS2,
        .max_forward_position_error_m =
            APP_BALANCE_MAX_FORWARD_POSITION_ERROR_M,
        .max_lateral_position_error_m =
            APP_BALANCE_MAX_LATERAL_POSITION_ERROR_M,
        .max_yaw_error_rad = APP_BALANCE_MAX_YAW_ERROR_RAD,
        .max_forward_current_a = APP_BALANCE_MAX_FORWARD_CURRENT_A,
        .max_lateral_current_a = APP_BALANCE_MAX_LATERAL_CURRENT_A,
        .max_yaw_current_a = APP_BALANCE_MAX_YAW_CURRENT_A,
        .forward_output_sign = APP_BALANCE_FORWARD_OUTPUT_SIGN,
        .lateral_output_sign = APP_BALANCE_LATERAL_OUTPUT_SIGN,
        .yaw_output_sign = APP_BALANCE_YAW_OUTPUT_SIGN,
    };
    const ctrl_collinear_mecanum_allocator_config_t allocator_config =
    {
        .wheel_offset_m =
        {
            APP_CONTROL_WHEEL_1_OFFSET_M,
            APP_CONTROL_WHEEL_2_OFFSET_M,
            APP_CONTROL_WHEEL_3_OFFSET_M,
            APP_CONTROL_WHEEL_4_OFFSET_M,
        },
        .roller_angle_rad =
        {
            APP_CONTROL_WHEEL_1_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_2_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_3_ROLLER_ANGLE_RAD,
            APP_CONTROL_WHEEL_4_ROLLER_ANGLE_RAD,
        },
        .yaw_scale_m = APP_CONTROL_YAW_SCALE_M,
        .max_motor_current_a = APP_BALANCE_MAX_CURRENT_A,
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

    if (!ctrl_balance_controller_init(&balance_controller,
                                      &balance_controller_config))
    {
        ESP_LOGE(TAG, "invalid balance controller configuration");
        vTaskDelete(NULL);
        return;
    }

    if (!ctrl_collinear_mecanum_allocator_init(&current_allocator,
                                                &allocator_config))
    {
        ESP_LOGE(TAG, "invalid current allocator configuration");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "200 Hz state estimator and controller started");

    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_loop_us = 0u;
    uint32_t handled_zero_lean_count = 0u;
    uint32_t applied_zero_lean_count = 0u;
    uint32_t rejected_zero_lean_count = 0u;
    float lean_trim_rad = APP_BALANCE_INITIAL_LEAN_TRIM_RAD;
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
        app_balance_safety_snapshot_t balance_safety_snapshot;
        safety_imu_status_t imu_status;
        est_collinear_mecanum_output_t odometry;
        float raw_lean_rad = 0.0f;
        float raw_lean_rate_rps = 0.0f;

        last_loop_us = loop_start_us;
        snapshot.timestamp_us = loop_start_us;
        snapshot.loop_count++;
        snapshot.input_fault_flags = CTRL_STATE_FAULT_NONE;
        memset(&snapshot.state, 0, sizeof(snapshot.state));
        memset(snapshot.desired_current_a,
               0,
               sizeof(snapshot.desired_current_a));
        snapshot.state.timestamp_us = loop_start_us;
        snapshot.lean_trim_rad = lean_trim_rad;
        snapshot.forward_velocity_reference_mps = 0.0f;
        snapshot.lateral_velocity_reference_mps = 0.0f;
        snapshot.yaw_rate_reference_rps = 0.0f;
        snapshot.forward_position_error_m = 0.0f;
        snapshot.lateral_position_error_m = 0.0f;
        snapshot.yaw_error_rad = 0.0f;
        snapshot.balance_lean_error_rad = 0.0f;
        snapshot.forward_current_a = 0.0f;
        snapshot.lateral_current_a = 0.0f;
        snapshot.yaw_current_a = 0.0f;
        snapshot.zero_lean_applied_count = applied_zero_lean_count;
        snapshot.zero_lean_rejected_count = rejected_zero_lean_count;
        snapshot.balance_command_valid = false;
        snapshot.balance_output_saturated = false;

        task_imu_get_snapshot(&imu_snapshot);
        task_motor_get_snapshot(&actuator_snapshot);
        task_supervisor_get_snapshot(&supervisor_snapshot);
        task_safety_get_balance_snapshot(&balance_safety_snapshot);
        (void)safety_imu_check(&imu_snapshot,
                               loop_start_us,
                               &imu_safety_config,
                               &imu_status);

        const bool lean_updated = app_balance_state_get_lean(
            &imu_snapshot,
            &imu_status,
            APP_CONTROL_LEAN_AXIS,
            &raw_lean_rad,
            &raw_lean_rate_rps);

        if (lean_updated &&
            (supervisor_snapshot.zero_lean_request_count !=
             handled_zero_lean_count))
        {
            handled_zero_lean_count =
                supervisor_snapshot.zero_lean_request_count;

            if ((supervisor_snapshot.mode == APP_SYSTEM_MODE_SAFE_IDLE) &&
                (fabsf(raw_lean_rad) <=
                 APP_BALANCE_ZERO_MAX_ABS_LEAN_RAD) &&
                (fabsf(raw_lean_rate_rps) <=
                 APP_BALANCE_ZERO_MAX_LEAN_RATE_RPS))
            {
                lean_trim_rad = raw_lean_rad;
                applied_zero_lean_count = handled_zero_lean_count;
                rejected_zero_lean_count = 0u;
                balance_reference_captured = false;
            }
            else
            {
                rejected_zero_lean_count++;
            }
        }

        if (lean_updated)
        {
            snapshot.state.lean_rad = raw_lean_rad - lean_trim_rad;
            snapshot.state.lean_rate_rps = raw_lean_rate_rps;
            snapshot.lean_trim_rad = lean_trim_rad;
            snapshot.zero_lean_applied_count = applied_zero_lean_count;
            snapshot.zero_lean_rejected_count = rejected_zero_lean_count;
        }

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

        if (snapshot.state.valid)
        {
            snapshot.state.lean_rad -= lean_trim_rad;
            snapshot.lean_trim_rad = lean_trim_rad;
            snapshot.zero_lean_applied_count = applied_zero_lean_count;
            snapshot.zero_lean_rejected_count = rejected_zero_lean_count;
        }

        if (supervisor_snapshot.mode == APP_SYSTEM_MODE_BALANCE_CONTROL)
        {
            const ctrl_balance_reference_t reference =
            {
                .forward_velocity_mps =
                    supervisor_snapshot.forward_input *
                    APP_MOTION_MAX_FORWARD_VELOCITY_MPS,
                .lateral_velocity_mps =
                    supervisor_snapshot.lateral_input *
                    APP_MOTION_MAX_LATERAL_VELOCITY_MPS,
                .yaw_rate_rps = supervisor_snapshot.yaw_input *
                    APP_MOTION_MAX_YAW_RATE_RPS,
            };
            ctrl_balance_controller_output_t balance_output;

            if (snapshot.state.valid &&
                (!balance_reference_captured ||
                 !balance_safety_snapshot.motion_allowed))
            {
                ctrl_balance_controller_reset(
                    &balance_controller,
                    &snapshot.state);
                balance_reference_captured = true;
            }

            if (balance_reference_captured &&
                ctrl_balance_controller_update(
                    &balance_controller,
                    &snapshot.state,
                    &reference,
                    dt_s,
                    &balance_output))
            {
                const float axis_current_a
                    [CTRL_COLLINEAR_MECANUM_AXIS_COUNT] =
                {
                    balance_output.forward_current_a,
                    balance_output.lateral_current_a,
                    balance_output.yaw_current_a,
                };
                bool allocator_saturated = false;

                snapshot.forward_velocity_reference_mps =
                    balance_output.forward_velocity_reference_mps;
                snapshot.lateral_velocity_reference_mps =
                    balance_output.lateral_velocity_reference_mps;
                snapshot.yaw_rate_reference_rps =
                    balance_output.yaw_rate_reference_rps;
                snapshot.forward_position_error_m =
                    balance_output.forward_position_error_m;
                snapshot.lateral_position_error_m =
                    balance_output.lateral_position_error_m;
                snapshot.yaw_error_rad = balance_output.yaw_error_rad;
                snapshot.balance_lean_error_rad =
                    balance_output.lean_error_rad;
                snapshot.forward_current_a =
                    balance_output.forward_current_a;
                snapshot.lateral_current_a =
                    balance_output.lateral_current_a;
                snapshot.yaw_current_a = balance_output.yaw_current_a;

                if (ctrl_collinear_mecanum_allocator_update(
                        &current_allocator,
                        axis_current_a,
                        snapshot.desired_current_a,
                        &allocator_saturated))
                {
                    snapshot.balance_output_saturated =
                        balance_output.saturated || allocator_saturated;
                    snapshot.balance_command_valid = true;
                }
            }
        }
        else
        {
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
            snapshot.balance_command_valid = false;
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
