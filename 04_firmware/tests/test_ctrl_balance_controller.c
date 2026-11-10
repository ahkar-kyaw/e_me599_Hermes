#include "control/ctrl_balance_controller.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-5f

static void fail(const char *message)
{
    (void)fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_near(float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        fail("unexpected controller output");
    }
}

int main(void)
{
    const ctrl_balance_controller_config_t config =
    {
        .lean_trim_rad = 0.0f,
        .lean_gain_a_per_rad = 2.0f,
        .lean_rate_gain_a_per_rps = 0.5f,
        .position_gain_a_per_m = 0.3f,
        .velocity_gain_a_per_mps = 0.2f,
        .position_integral_gain_a_per_m_s = 0.1f,
        .position_integral_limit_m_s = 1.0f,
        .max_current_a = 1.0f,
        .output_sign = 1.0f,
    };
    ctrl_balance_controller_t controller;
    ctrl_balance_controller_output_t output;
    balance_state_t state;

    memset(&state, 0, sizeof(state));
    state.valid = true;

    if (!ctrl_balance_controller_init(&controller, &config))
    {
        fail("controller initialization failed");
    }

    ctrl_balance_controller_reset(&controller, 1.0f);
    state.position_x_m = 1.1f;
    state.velocity_x_mps = 0.2f;
    state.lean_rad = 0.2f;
    state.lean_rate_rps = 0.1f;

    if (!ctrl_balance_controller_update(&controller,
                                        &state,
                                        0.01f,
                                        &output))
    {
        fail("valid controller update failed");
    }

    expect_near(output.position_error_m, 0.1f);
    expect_near(output.position_integral_m_s, 0.001f);
    expect_near(output.common_current_a, 0.3799f);

    ctrl_balance_controller_reset(&controller, 0.0f);
    memset(&state, 0, sizeof(state));
    state.valid = true;
    state.position_x_m = 1.0f;

    (void)ctrl_balance_controller_update(&controller,
                                         &state,
                                         0.01f,
                                         &output);

    if (output.common_current_a >= 0.0f)
    {
        fail("positive position error did not command recovery current");
    }

    state.position_x_m = 0.0f;
    state.lean_rad = 1.0f;
    (void)ctrl_balance_controller_update(&controller,
                                         &state,
                                         0.01f,
                                         &output);

    expect_near(output.common_current_a, 1.0f);

    if (!output.saturated)
    {
        fail("current saturation was not reported");
    }

    return EXIT_SUCCESS;
}
