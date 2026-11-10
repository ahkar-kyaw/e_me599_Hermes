#include "control/ctrl_torque_input.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool ctrl_torque_input_config_valid(
    const ctrl_torque_input_config_t *config);
static float ctrl_torque_input_target(
    const ctrl_torque_input_config_t *config,
    uint16_t channel_value);

bool ctrl_torque_input_init(
    ctrl_torque_input_t *input,
    const ctrl_torque_input_config_t *config)
{
    if ((input == NULL) || !ctrl_torque_input_config_valid(config))
    {
        return false;
    }

    memset(input, 0, sizeof(*input));
    input->config = *config;
    input->initialized = true;
    return true;
}

void ctrl_torque_input_reset(ctrl_torque_input_t *input)
{
    if ((input == NULL) || !input->initialized)
    {
        return;
    }

    input->current_a = 0.0f;
}

bool ctrl_torque_input_update(ctrl_torque_input_t *input,
                              uint16_t channel_value,
                              float dt_s,
                              float *current_a)
{
    if ((input == NULL) || !input->initialized ||
        (current_a == NULL) || !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return false;
    }

    const float target = ctrl_torque_input_target(
        &input->config,
        channel_value);
    const float maximum_step = input->config.slew_rate_a_per_s * dt_s;
    const float error = target - input->current_a;

    if (error > maximum_step)
    {
        input->current_a += maximum_step;
    }
    else if (error < -maximum_step)
    {
        input->current_a -= maximum_step;
    }
    else
    {
        input->current_a = target;
    }

    *current_a = input->current_a;
    return true;
}

static bool ctrl_torque_input_config_valid(
    const ctrl_torque_input_config_t *config)
{
    return (config != NULL) &&
           (config->channel_min < config->neutral_low) &&
           (config->neutral_low <= config->neutral_high) &&
           (config->neutral_high < config->channel_max) &&
           isfinite(config->max_current_a) &&
           isfinite(config->slew_rate_a_per_s) &&
           (config->max_current_a > 0.0f) &&
           (config->slew_rate_a_per_s > 0.0f);
}

static float ctrl_torque_input_target(
    const ctrl_torque_input_config_t *config,
    uint16_t channel_value)
{
    float normalized = 0.0f;

    if (channel_value < config->neutral_low)
    {
        const uint16_t bounded = (channel_value < config->channel_min) ?
            config->channel_min : channel_value;

        normalized = -((float)config->neutral_low - (float)bounded) /
            ((float)config->neutral_low - (float)config->channel_min);
    }
    else if (channel_value > config->neutral_high)
    {
        const uint16_t bounded = (channel_value > config->channel_max) ?
            config->channel_max : channel_value;

        normalized = ((float)bounded - (float)config->neutral_high) /
            ((float)config->channel_max - (float)config->neutral_high);
    }

    if (!config->positive_high)
    {
        normalized = -normalized;
    }

    return normalized * config->max_current_a;
}
