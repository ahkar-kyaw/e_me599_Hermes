#include "app/app_operator_controls.h"

#include <stddef.h>
#include <string.h>

static bool app_operator_controls_switch_active(
    uint16_t value,
    bool current_active,
    bool active_high,
    uint16_t off_threshold,
    uint16_t on_threshold);
static bool app_operator_controls_config_valid(
    const app_operator_controls_config_t *config);
static bool app_operator_controls_channels_valid(
    const app_operator_controls_t *controls,
    const rc_snapshot_t *snapshot);

bool app_operator_controls_init(
    app_operator_controls_t *controls,
    const app_operator_controls_config_t *config)
{
    if ((controls == NULL) ||
        !app_operator_controls_config_valid(config))
    {
        return false;
    }

    memset(controls, 0, sizeof(*controls));
    controls->config = *config;
    return true;
}

void app_operator_controls_update(
    app_operator_controls_t *controls,
    const rc_snapshot_t *snapshot,
    bool link_usable,
    app_operator_controls_state_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }

    if ((controls == NULL) || (snapshot == NULL) || (state == NULL) ||
        !link_usable ||
        !app_operator_controls_channels_valid(controls, snapshot))
    {
        if (controls != NULL)
        {
            controls->initialized = false;
            controls->master_active = false;
            controls->balance_active = false;
            controls->zero_lean_active = false;
        }

        return;
    }

    const bool previous_zero_active = controls->zero_lean_active;
    const app_operator_controls_config_t *config = &controls->config;

    controls->master_active = app_operator_controls_switch_active(
        snapshot->channel[config->master_channel],
        controls->master_active,
        config->master_active_high,
        config->switch_off_threshold,
        config->switch_on_threshold);
    controls->balance_active = app_operator_controls_switch_active(
        snapshot->channel[config->balance_channel],
        controls->balance_active,
        config->balance_active_high,
        config->switch_off_threshold,
        config->switch_on_threshold);
    controls->zero_lean_active = app_operator_controls_switch_active(
        snapshot->channel[config->zero_lean_channel],
        controls->zero_lean_active,
        config->zero_lean_active_high,
        config->switch_off_threshold,
        config->switch_on_threshold);

    state->link_usable = true;
    state->master_enabled = controls->master_active;
    state->balance_enabled = controls->balance_active;
    state->zero_lean_requested = controls->initialized &&
        !previous_zero_active && controls->zero_lean_active;
    controls->initialized = true;
}

static bool app_operator_controls_switch_active(
    uint16_t value,
    bool current_active,
    bool active_high,
    uint16_t off_threshold,
    uint16_t on_threshold)
{
    if (active_high)
    {
        if (!current_active && (value >= on_threshold))
        {
            return true;
        }

        if (current_active && (value <= off_threshold))
        {
            return false;
        }
    }
    else
    {
        if (!current_active && (value <= off_threshold))
        {
            return true;
        }

        if (current_active && (value >= on_threshold))
        {
            return false;
        }
    }

    return current_active;
}

static bool app_operator_controls_config_valid(
    const app_operator_controls_config_t *config)
{
    return (config != NULL) &&
           (config->master_channel < APP_RC_CHANNEL_COUNT) &&
           (config->balance_channel < APP_RC_CHANNEL_COUNT) &&
           (config->zero_lean_channel < APP_RC_CHANNEL_COUNT) &&
           (config->channel_min < config->channel_max) &&
           (config->switch_off_threshold >= config->channel_min) &&
           (config->switch_off_threshold < config->switch_on_threshold) &&
           (config->switch_on_threshold <= config->channel_max);
}

static bool app_operator_controls_channels_valid(
    const app_operator_controls_t *controls,
    const rc_snapshot_t *snapshot)
{
    const uint8_t channel[] =
    {
        controls->config.master_channel,
        controls->config.balance_channel,
        controls->config.zero_lean_channel,
    };

    for (size_t i = 0u; i < (sizeof(channel) / sizeof(channel[0])); i++)
    {
        const uint16_t value = snapshot->channel[channel[i]];

        if ((value < controls->config.channel_min) ||
            (value > controls->config.channel_max))
        {
            return false;
        }
    }

    return true;
}
