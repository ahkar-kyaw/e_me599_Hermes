#include "control/ctrl_balance_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CTRL_BALANCE_PI     3.1415926536f
#define CTRL_BALANCE_TWO_PI 6.2831853072f

static float ctrl_balance_clamp(float value, float limit);
static float ctrl_balance_slew(float current,
                               float target,
                               float rate_limit,
                               float dt_s);
static float ctrl_balance_wrap_pi(float angle_rad);
static float ctrl_balance_axis_current(float current_without_integral_a,
                                       float error,
                                       float integral_gain,
                                       float integral_limit,
                                       float output_sign,
                                       float current_limit,
                                       float dt_s,
                                       float *integral,
                                       bool *saturated);
static bool ctrl_balance_config_valid(
    const ctrl_balance_controller_config_t *config);
static bool ctrl_balance_state_valid(const balance_state_t *state);
static bool ctrl_balance_reference_valid(
    const ctrl_balance_reference_t *reference);

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
    const balance_state_t *state)
{
    if ((controller == NULL) || !controller->initialized ||
        !ctrl_balance_state_valid(state))
    {
        return;
    }

    controller->reference_position_x_m = state->position_x_m;
    controller->reference_position_y_m = state->position_y_m;
    controller->reference_yaw_rad = state->yaw_rad;
    controller->forward_velocity_reference_mps = 0.0f;
    controller->lateral_velocity_reference_mps = 0.0f;
    controller->yaw_rate_reference_rps = 0.0f;
    controller->forward_integral_m_s = 0.0f;
    controller->lateral_integral_m_s = 0.0f;
    controller->yaw_integral_rad_s = 0.0f;
    controller->reference_valid = true;
}

bool ctrl_balance_controller_update(
    ctrl_balance_controller_t *controller,
    const balance_state_t *state,
    const ctrl_balance_reference_t *reference,
    float dt_s,
    ctrl_balance_controller_output_t *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
    }

    if ((controller == NULL) || !controller->initialized ||
        !controller->reference_valid || !ctrl_balance_state_valid(state) ||
        !ctrl_balance_reference_valid(reference) || (output == NULL) ||
        !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return false;
    }

    const ctrl_balance_controller_config_t *config = &controller->config;

    controller->forward_velocity_reference_mps = ctrl_balance_slew(
        controller->forward_velocity_reference_mps,
        reference->forward_velocity_mps,
        config->max_forward_accel_mps2,
        dt_s);
    controller->lateral_velocity_reference_mps = ctrl_balance_slew(
        controller->lateral_velocity_reference_mps,
        reference->lateral_velocity_mps,
        config->max_lateral_accel_mps2,
        dt_s);
    controller->yaw_rate_reference_rps = ctrl_balance_slew(
        controller->yaw_rate_reference_rps,
        reference->yaw_rate_rps,
        config->max_yaw_accel_rps2,
        dt_s);

    const float cosine_yaw = cosf(state->yaw_rad);
    const float sine_yaw = sinf(state->yaw_rad);
    controller->reference_position_x_m +=
        ((cosine_yaw * controller->forward_velocity_reference_mps) -
         (sine_yaw * controller->lateral_velocity_reference_mps)) * dt_s;
    controller->reference_position_y_m +=
        ((sine_yaw * controller->forward_velocity_reference_mps) +
         (cosine_yaw * controller->lateral_velocity_reference_mps)) * dt_s;
    controller->reference_yaw_rad = ctrl_balance_wrap_pi(
        controller->reference_yaw_rad +
        (controller->yaw_rate_reference_rps * dt_s));

    const float world_error_x_m = state->position_x_m -
        controller->reference_position_x_m;
    const float world_error_y_m = state->position_y_m -
        controller->reference_position_y_m;
    const float forward_error_unlimited_m =
        (cosine_yaw * world_error_x_m) +
        (sine_yaw * world_error_y_m);
    const float lateral_error_unlimited_m =
        (-sine_yaw * world_error_x_m) +
        (cosine_yaw * world_error_y_m);
    const float yaw_error_unlimited_rad = ctrl_balance_wrap_pi(
        state->yaw_rad - controller->reference_yaw_rad);
    const float forward_error_m = ctrl_balance_clamp(
        forward_error_unlimited_m,
        config->max_forward_position_error_m);
    const float lateral_error_m = ctrl_balance_clamp(
        lateral_error_unlimited_m,
        config->max_lateral_position_error_m);
    const float yaw_error_rad = ctrl_balance_clamp(
        yaw_error_unlimited_rad,
        config->max_yaw_error_rad);

    if ((forward_error_m != forward_error_unlimited_m) ||
        (lateral_error_m != lateral_error_unlimited_m))
    {
        const float limited_world_x_m =
            (cosine_yaw * forward_error_m) -
            (sine_yaw * lateral_error_m);
        const float limited_world_y_m =
            (sine_yaw * forward_error_m) +
            (cosine_yaw * lateral_error_m);

        controller->reference_position_x_m =
            state->position_x_m - limited_world_x_m;
        controller->reference_position_y_m =
            state->position_y_m - limited_world_y_m;
    }

    if (yaw_error_rad != yaw_error_unlimited_rad)
    {
        controller->reference_yaw_rad = ctrl_balance_wrap_pi(
            state->yaw_rad - yaw_error_rad);
    }

    const float body_forward_mps =
        (cosine_yaw * state->velocity_x_mps) +
        (sine_yaw * state->velocity_y_mps);
    const float body_left_mps =
        (-sine_yaw * state->velocity_x_mps) +
        (cosine_yaw * state->velocity_y_mps);
    const float forward_current_without_integral_a =
        config->forward_output_sign *
        ((config->lean_gain_a_per_rad * state->lean_rad) +
         (config->lean_rate_gain_a_per_rps * state->lean_rate_rps) -
         (config->forward_position_gain_a_per_m * forward_error_m) -
         (config->forward_velocity_gain_a_per_mps *
          (body_forward_mps -
           controller->forward_velocity_reference_mps)));
    const float lateral_current_without_integral_a =
        config->lateral_output_sign *
        (-(config->lateral_position_gain_a_per_m * lateral_error_m) -
         (config->lateral_velocity_gain_a_per_mps *
          (body_left_mps - controller->lateral_velocity_reference_mps)));
    const float yaw_current_without_integral_a =
        config->yaw_output_sign *
        (-(config->yaw_position_gain_a_per_rad * yaw_error_rad) -
         (config->yaw_rate_gain_a_per_rps *
          (state->yaw_rate_rps - controller->yaw_rate_reference_rps)));
    bool forward_saturated = false;
    bool lateral_saturated = false;
    bool yaw_saturated = false;

    output->forward_current_a = ctrl_balance_axis_current(
        forward_current_without_integral_a,
        forward_error_m,
        config->forward_integral_gain_a_per_m_s,
        config->forward_integral_limit_m_s,
        config->forward_output_sign,
        config->max_forward_current_a,
        dt_s,
        &controller->forward_integral_m_s,
        &forward_saturated);
    output->lateral_current_a = ctrl_balance_axis_current(
        lateral_current_without_integral_a,
        lateral_error_m,
        config->lateral_integral_gain_a_per_m_s,
        config->lateral_integral_limit_m_s,
        config->lateral_output_sign,
        config->max_lateral_current_a,
        dt_s,
        &controller->lateral_integral_m_s,
        &lateral_saturated);
    output->yaw_current_a = ctrl_balance_axis_current(
        yaw_current_without_integral_a,
        yaw_error_rad,
        config->yaw_integral_gain_a_per_rad_s,
        config->yaw_integral_limit_rad_s,
        config->yaw_output_sign,
        config->max_yaw_current_a,
        dt_s,
        &controller->yaw_integral_rad_s,
        &yaw_saturated);

    output->reference_position_x_m = controller->reference_position_x_m;
    output->reference_position_y_m = controller->reference_position_y_m;
    output->reference_yaw_rad = controller->reference_yaw_rad;
    output->forward_velocity_reference_mps =
        controller->forward_velocity_reference_mps;
    output->lateral_velocity_reference_mps =
        controller->lateral_velocity_reference_mps;
    output->yaw_rate_reference_rps = controller->yaw_rate_reference_rps;
    output->forward_position_error_m = forward_error_m;
    output->lateral_position_error_m = lateral_error_m;
    output->yaw_error_rad = yaw_error_rad;
    output->forward_integral_m_s = controller->forward_integral_m_s;
    output->lateral_integral_m_s = controller->lateral_integral_m_s;
    output->yaw_integral_rad_s = controller->yaw_integral_rad_s;
    output->lean_error_rad = state->lean_rad;
    output->saturated = forward_saturated || lateral_saturated ||
        yaw_saturated;
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

static float ctrl_balance_slew(float current,
                               float target,
                               float rate_limit,
                               float dt_s)
{
    const float maximum_change = rate_limit * dt_s;
    return current + ctrl_balance_clamp(target - current,
                                        maximum_change);
}

static float ctrl_balance_wrap_pi(float angle_rad)
{
    while (angle_rad > CTRL_BALANCE_PI)
    {
        angle_rad -= CTRL_BALANCE_TWO_PI;
    }

    while (angle_rad < -CTRL_BALANCE_PI)
    {
        angle_rad += CTRL_BALANCE_TWO_PI;
    }

    return angle_rad;
}

static float ctrl_balance_axis_current(float current_without_integral_a,
                                       float error,
                                       float integral_gain,
                                       float integral_limit,
                                       float output_sign,
                                       float current_limit,
                                       float dt_s,
                                       float *integral,
                                       bool *saturated)
{
    const float candidate_integral = ctrl_balance_clamp(
        *integral + (error * dt_s),
        integral_limit);
    const float candidate_current_a = current_without_integral_a -
        (output_sign * integral_gain * candidate_integral);
    const float limited_candidate_a = ctrl_balance_clamp(
        candidate_current_a,
        current_limit);
    const bool candidate_saturated =
        candidate_current_a != limited_candidate_a;
    const float integral_current_delta =
        -output_sign * integral_gain * error * dt_s;

    if (!candidate_saturated ||
        ((candidate_current_a * integral_current_delta) < 0.0f))
    {
        *integral = candidate_integral;
    }

    const float raw_current_a = current_without_integral_a -
        (output_sign * integral_gain * *integral);
    const float limited_current_a = ctrl_balance_clamp(
        raw_current_a,
        current_limit);
    *saturated = raw_current_a != limited_current_a;
    return limited_current_a;
}

static bool ctrl_balance_config_valid(
    const ctrl_balance_controller_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    const float nonnegative[] =
    {
        config->lean_gain_a_per_rad,
        config->lean_rate_gain_a_per_rps,
        config->forward_position_gain_a_per_m,
        config->forward_velocity_gain_a_per_mps,
        config->forward_integral_gain_a_per_m_s,
        config->lateral_position_gain_a_per_m,
        config->lateral_velocity_gain_a_per_mps,
        config->lateral_integral_gain_a_per_m_s,
        config->yaw_position_gain_a_per_rad,
        config->yaw_rate_gain_a_per_rps,
        config->yaw_integral_gain_a_per_rad_s,
    };
    const float positive[] =
    {
        config->forward_integral_limit_m_s,
        config->lateral_integral_limit_m_s,
        config->yaw_integral_limit_rad_s,
        config->max_forward_accel_mps2,
        config->max_lateral_accel_mps2,
        config->max_yaw_accel_rps2,
        config->max_forward_position_error_m,
        config->max_lateral_position_error_m,
        config->max_yaw_error_rad,
        config->max_forward_current_a,
        config->max_lateral_current_a,
        config->max_yaw_current_a,
    };

    for (size_t index = 0u;
         index < (sizeof(nonnegative) / sizeof(nonnegative[0]));
         index++)
    {
        if (!isfinite(nonnegative[index]) || (nonnegative[index] < 0.0f))
        {
            return false;
        }
    }

    for (size_t index = 0u;
         index < (sizeof(positive) / sizeof(positive[0]));
         index++)
    {
        if (!isfinite(positive[index]) || (positive[index] <= 0.0f))
        {
            return false;
        }
    }

    return isfinite(config->forward_output_sign) &&
           isfinite(config->lateral_output_sign) &&
           isfinite(config->yaw_output_sign) &&
           ((config->forward_output_sign == -1.0f) ||
            (config->forward_output_sign == 1.0f)) &&
           ((config->lateral_output_sign == -1.0f) ||
            (config->lateral_output_sign == 1.0f)) &&
           ((config->yaw_output_sign == -1.0f) ||
            (config->yaw_output_sign == 1.0f));
}

static bool ctrl_balance_state_valid(const balance_state_t *state)
{
    return (state != NULL) && state->valid &&
           isfinite(state->position_x_m) &&
           isfinite(state->position_y_m) &&
           isfinite(state->yaw_rad) &&
           isfinite(state->lean_rad) &&
           isfinite(state->velocity_x_mps) &&
           isfinite(state->velocity_y_mps) &&
           isfinite(state->yaw_rate_rps) &&
           isfinite(state->lean_rate_rps);
}

static bool ctrl_balance_reference_valid(
    const ctrl_balance_reference_t *reference)
{
    return (reference != NULL) &&
           isfinite(reference->forward_velocity_mps) &&
           isfinite(reference->lateral_velocity_mps) &&
           isfinite(reference->yaw_rate_rps);
}
