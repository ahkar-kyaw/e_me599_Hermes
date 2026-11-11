#ifndef CTRL_BALANCE_TYPES_H
#define CTRL_BALANCE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_BALANCE_ACTUATOR_COUNT 4u

typedef enum
{
    CTRL_BALANCE_LEAN_ROLL = 0,
    CTRL_BALANCE_LEAN_PITCH
} ctrl_balance_lean_axis_t;

typedef enum
{
    CTRL_STATE_FAULT_NONE      = 0u,
    CTRL_STATE_FAULT_IMU       = 1u << 0,
    CTRL_STATE_FAULT_ACTUATOR  = 1u << 1,
    CTRL_STATE_FAULT_TIMING    = 1u << 2,
    CTRL_STATE_FAULT_ESTIMATOR = 1u << 3
} ctrl_state_fault_t;

typedef struct
{
    bool valid;
    uint64_t timestamp_us;

    float position_x_m;
    float position_y_m;
    float yaw_rad;
    float lean_rad;

    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_rps;
    float lean_rate_rps;
} balance_state_t;

typedef struct
{
    balance_state_t state;

    float desired_current_a[CTRL_BALANCE_ACTUATOR_COUNT];
    float lean_trim_rad;
    float balance_reference_position_x_m;
    float balance_position_error_m;
    float balance_position_integral_m_s;
    float balance_lean_error_rad;
    float balance_common_current_a;

    uint64_t timestamp_us;
    uint32_t loop_count;
    uint32_t invalid_input_count;
    uint32_t overrun_count;
    uint32_t execution_time_us;
    uint32_t max_execution_time_us;
    uint32_t zero_lean_applied_count;
    uint32_t zero_lean_rejected_count;
    uint32_t input_fault_flags;
    bool balance_command_valid;
    bool balance_output_saturated;
} control_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
