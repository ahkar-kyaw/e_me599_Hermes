#include "app/app_operator_controls.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fail(const char *message)
{
    (void)fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

int main(void)
{
    const app_operator_controls_config_t config =
    {
        .master_channel = 4u,
        .balance_channel = 5u,
        .zero_lean_channel = 6u,
        .forward_channel = 2u,
        .lateral_channel = 0u,
        .yaw_channel = 3u,
        .channel_min = 172u,
        .channel_max = 1811u,
        .input_min = 191u,
        .input_mid = 992u,
        .input_max = 1792u,
        .input_deadband = 40u,
        .switch_off_threshold = 600u,
        .switch_on_threshold = 1384u,
        .master_active_high = true,
        .balance_active_high = true,
        .zero_lean_active_high = true,
        .forward_inverted = false,
        .lateral_inverted = false,
        .yaw_inverted = true,
    };
    app_operator_controls_t controls;
    app_operator_controls_state_t state;
    rc_snapshot_t rc;

    memset(&rc, 0, sizeof(rc));
    rc.channel[4] = 1792u;
    rc.channel[5] = 1792u;
    rc.channel[6] = 191u;
    rc.channel[2] = 1792u;
    rc.channel[0] = 191u;
    rc.channel[3] = 1792u;

    if (!app_operator_controls_init(&controls, &config))
    {
        fail("operator control initialization failed");
    }

    app_operator_controls_update(&controls, &rc, true, &state);

    if (!state.link_usable || !state.master_enabled ||
        !state.balance_enabled || state.zero_lean_requested)
    {
        fail("initial toggle levels were not applied directly");
    }

    if ((state.forward_input != 1.0f) ||
        (state.lateral_input != -1.0f) ||
        (state.yaw_input != -1.0f))
    {
        fail("motion channels were not normalized");
    }

    rc.channel[2] = 1000u;
    app_operator_controls_update(&controls, &rc, true, &state);

    if (state.forward_input != 0.0f)
    {
        fail("motion deadband was not applied");
    }

    rc.channel[6] = 1792u;
    app_operator_controls_update(&controls, &rc, true, &state);

    if (!state.zero_lean_requested)
    {
        fail("zero-lean rising edge was not reported");
    }

    app_operator_controls_update(&controls, &rc, true, &state);

    if (state.zero_lean_requested)
    {
        fail("held zero-lean switch repeated the request");
    }

    app_operator_controls_update(&controls, &rc, false, &state);

    if (state.link_usable || state.master_enabled ||
        state.balance_enabled)
    {
        fail("lost link did not disable controls");
    }

    return EXIT_SUCCESS;
}
