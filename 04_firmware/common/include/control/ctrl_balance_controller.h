#ifndef CTRL_BALANCE_CONTROLLER_H
#define CTRL_BALANCE_CONTROLLER_H

#include "control/ctrl_balance_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float lean_gain_a_per_rad;
    float lean_rate_gain_a_per_rps;
    float position_gain_a_per_m;
    float velocity_gain_a_per_mps;
    float position_integral_gain_a_per_m_s;
    float position_integral_limit_m_s;
    float max_current_a;
    float output_sign;
} ctrl_balance_controller_config_t;

typedef struct
{
    ctrl_balance_controller_config_t config;
    float reference_position_x_m;
    float position_integral_m_s;
    bool reference_valid;
    bool initialized;
} ctrl_balance_controller_t;

typedef struct
{
    float reference_position_x_m;
    float position_error_m;
    float position_integral_m_s;
    float lean_error_rad;
    float common_current_a;
    bool saturated;
    bool valid;
} ctrl_balance_controller_output_t;

bool ctrl_balance_controller_init(
    ctrl_balance_controller_t *controller,
    const ctrl_balance_controller_config_t *config);

void ctrl_balance_controller_reset(
    ctrl_balance_controller_t *controller,
    float reference_position_x_m);

bool ctrl_balance_controller_update(
    ctrl_balance_controller_t *controller,
    const balance_state_t *state,
    float dt_s,
    ctrl_balance_controller_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
