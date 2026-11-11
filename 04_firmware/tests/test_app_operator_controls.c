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
        .channel_min = 172u,
        .channel_max = 1811u,
        .switch_off_threshold = 600u,
        .switch_on_threshold = 1384u,
        .master_active_high = true,
        .balance_active_high = true,
        .zero_lean_active_high = true,
    };
    app_operator_controls_t controls;
    app_operator_controls_state_t state;
    rc_snapshot_t rc;

    memset(&rc, 0, sizeof(rc));
    rc.channel[4] = 1792u;
    rc.channel[5] = 1792u;
    rc.channel[6] = 191u;

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
