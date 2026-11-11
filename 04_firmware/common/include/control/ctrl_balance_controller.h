#ifndef CTRL_BALANCE_CONTROLLER_H
#define CTRL_BALANCE_CONTROLLER_H

#include "control/ctrl_balance_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float forward_velocity_mps;
    float lateral_velocity_mps;
    float yaw_rate_rps;
} ctrl_balance_reference_t;

typedef struct
{
    float lean_gain_a_per_rad;
    float lean_rate_gain_a_per_rps;
    float forward_position_gain_a_per_m;
    float forward_velocity_gain_a_per_mps;
    float forward_integral_gain_a_per_m_s;
    float forward_integral_limit_m_s;
    float lateral_position_gain_a_per_m;
    float lateral_velocity_gain_a_per_mps;
    float lateral_integral_gain_a_per_m_s;
    float lateral_integral_limit_m_s;
    float yaw_position_gain_a_per_rad;
    float yaw_rate_gain_a_per_rps;
    float yaw_integral_gain_a_per_rad_s;
    float yaw_integral_limit_rad_s;
    float max_forward_accel_mps2;
    float max_lateral_accel_mps2;
    float max_yaw_accel_rps2;
    float max_forward_position_error_m;
    float max_lateral_position_error_m;
    float max_yaw_error_rad;
    float max_forward_current_a;
    float max_lateral_current_a;
    float max_yaw_current_a;
    float forward_output_sign;
    float lateral_output_sign;
    float yaw_output_sign;
} ctrl_balance_controller_config_t;

typedef struct
{
    ctrl_balance_controller_config_t config;
    float reference_position_x_m;
    float reference_position_y_m;
    float reference_yaw_rad;
    float forward_velocity_reference_mps;
    float lateral_velocity_reference_mps;
    float yaw_rate_reference_rps;
    float forward_integral_m_s;
    float lateral_integral_m_s;
    float yaw_integral_rad_s;
    bool reference_valid;
    bool initialized;
} ctrl_balance_controller_t;

typedef struct
{
    float reference_position_x_m;
    float reference_position_y_m;
    float reference_yaw_rad;
    float forward_velocity_reference_mps;
    float lateral_velocity_reference_mps;
    float yaw_rate_reference_rps;
    float forward_position_error_m;
    float lateral_position_error_m;
    float yaw_error_rad;
    float forward_integral_m_s;
    float lateral_integral_m_s;
    float yaw_integral_rad_s;
    float lean_error_rad;
    float forward_current_a;
    float lateral_current_a;
    float yaw_current_a;
    bool saturated;
    bool valid;
} ctrl_balance_controller_output_t;

bool ctrl_balance_controller_init(
    ctrl_balance_controller_t *controller,
    const ctrl_balance_controller_config_t *config);

void ctrl_balance_controller_reset(
    ctrl_balance_controller_t *controller,
    const balance_state_t *state);

bool ctrl_balance_controller_update(
    ctrl_balance_controller_t *controller,
    const balance_state_t *state,
    const ctrl_balance_reference_t *reference,
    float dt_s,
    ctrl_balance_controller_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
