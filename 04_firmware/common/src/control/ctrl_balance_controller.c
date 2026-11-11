#include "control/ctrl_balance_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float ctrl_balance_clamp(float value, float limit);
static bool ctrl_balance_config_valid(
    const ctrl_balance_controller_config_t *config);

bool ctrl_balance_controller_init(
    ctrl_balance_controller_t *controller,
    const ctrl_balance_controller_config_t *config)
{
    if ((controller == NULL) || !ctrl_balance_config_valid(config))
    {
        return false;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->initialized = true;
    return true;
}

void ctrl_balance_controller_reset(
    ctrl_balance_controller_t *controller,
    float reference_position_x_m)
{
    if ((controller == NULL) || !controller->initialized ||
        !isfinite(reference_position_x_m))
    {
        return;
    }

    controller->reference_position_x_m = reference_position_x_m;
    controller->position_integral_m_s = 0.0f;
    controller->reference_valid = true;
}

bool ctrl_balance_controller_update(
    ctrl_balance_controller_t *controller,
    const balance_state_t *state,
    float dt_s,
    ctrl_balance_controller_output_t *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
    }

    if ((controller == NULL) || !controller->initialized ||
        !controller->reference_valid || (state == NULL) ||
        (output == NULL) || !state->valid ||
        !isfinite(dt_s) || (dt_s <= 0.0f) ||
        !isfinite(state->position_x_m) ||
        !isfinite(state->velocity_x_mps) ||
        !isfinite(state->lean_rad) ||
        !isfinite(state->lean_rate_rps))
    {
        return false;
    }

    const float position_error_m = state->position_x_m -
        controller->reference_position_x_m;
    const float lean_error_rad = state->lean_rad;
    const float integral_candidate = ctrl_balance_clamp(
        controller->position_integral_m_s + (position_error_m * dt_s),
        controller->config.position_integral_limit_m_s);
    const float current_without_integral_a =
        controller->config.output_sign *
        ((controller->config.lean_gain_a_per_rad * lean_error_rad) +
         (controller->config.lean_rate_gain_a_per_rps *
          state->lean_rate_rps) -
         (controller->config.position_gain_a_per_m * position_error_m) -
         (controller->config.velocity_gain_a_per_mps *
          state->velocity_x_mps));
    const float candidate_current_a = current_without_integral_a -
        (controller->config.output_sign *
         controller->config.position_integral_gain_a_per_m_s *
         integral_candidate);
    const float candidate_limited_current_a = ctrl_balance_clamp(
        candidate_current_a,
        controller->config.max_current_a);

    if (!isfinite(candidate_current_a))
    {
        return false;
    }

    const bool candidate_saturated =
        candidate_current_a != candidate_limited_current_a;
    const float integral_current_delta =
        -controller->config.output_sign *
        controller->config.position_integral_gain_a_per_m_s *
        position_error_m * dt_s;
    const bool integration_reduces_saturation =
        candidate_saturated &&
        ((candidate_current_a * integral_current_delta) < 0.0f);

    if (!candidate_saturated || integration_reduces_saturation)
    {
        controller->position_integral_m_s = integral_candidate;
    }

    const float raw_current_a = current_without_integral_a -
        (controller->config.output_sign *
         controller->config.position_integral_gain_a_per_m_s *
         controller->position_integral_m_s);
    const float limited_current_a = ctrl_balance_clamp(
        raw_current_a,
        controller->config.max_current_a);
    const bool saturated = raw_current_a != limited_current_a;

    output->reference_position_x_m =
        controller->reference_position_x_m;
    output->position_error_m = position_error_m;
    output->position_integral_m_s =
        controller->position_integral_m_s;
    output->lean_error_rad = lean_error_rad;
    output->common_current_a = limited_current_a;
    output->saturated = saturated;
    output->valid = true;
    return true;
}

static float ctrl_balance_clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

static bool ctrl_balance_config_valid(
    const ctrl_balance_controller_config_t *config)
{
    return (config != NULL) &&
           isfinite(config->lean_gain_a_per_rad) &&
           isfinite(config->lean_rate_gain_a_per_rps) &&
           isfinite(config->position_gain_a_per_m) &&
           isfinite(config->velocity_gain_a_per_mps) &&
           isfinite(config->position_integral_gain_a_per_m_s) &&
           isfinite(config->position_integral_limit_m_s) &&
           isfinite(config->max_current_a) &&
           isfinite(config->output_sign) &&
           (config->lean_gain_a_per_rad >= 0.0f) &&
           (config->lean_rate_gain_a_per_rps >= 0.0f) &&
           (config->position_gain_a_per_m >= 0.0f) &&
           (config->velocity_gain_a_per_mps >= 0.0f) &&
           (config->position_integral_gain_a_per_m_s >= 0.0f) &&
           (config->position_integral_limit_m_s > 0.0f) &&
           (config->max_current_a > 0.0f) &&
           ((config->output_sign == -1.0f) ||
            (config->output_sign == 1.0f));
}
