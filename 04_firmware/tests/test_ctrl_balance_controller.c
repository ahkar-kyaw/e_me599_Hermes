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

static ctrl_balance_controller_config_t make_config(void)
{
    const ctrl_balance_controller_config_t config =
    {
        .lean_gain_a_per_rad = 2.0f,
        .lean_rate_gain_a_per_rps = 0.5f,
        .forward_position_gain_a_per_m = 0.3f,
        .forward_velocity_gain_a_per_mps = 0.2f,
        .forward_integral_gain_a_per_m_s = 0.1f,
        .forward_integral_limit_m_s = 1.0f,
        .lateral_position_gain_a_per_m = 0.2f,
        .lateral_velocity_gain_a_per_mps = 0.15f,
        .lateral_integral_gain_a_per_m_s = 0.0f,
        .lateral_integral_limit_m_s = 1.0f,
        .yaw_position_gain_a_per_rad = 0.15f,
        .yaw_rate_gain_a_per_rps = 0.1f,
        .yaw_integral_gain_a_per_rad_s = 0.0f,
        .yaw_integral_limit_rad_s = 1.0f,
        .max_forward_accel_mps2 = 1.0f,
        .max_lateral_accel_mps2 = 2.0f,
        .max_yaw_accel_rps2 = 3.0f,
        .max_forward_position_error_m = 0.35f,
        .max_lateral_position_error_m = 0.35f,
        .max_yaw_error_rad = 0.5f,
        .max_forward_current_a = 1.0f,
        .max_lateral_current_a = 0.35f,
        .max_yaw_current_a = 0.3f,
        .forward_output_sign = 1.0f,
        .lateral_output_sign = 1.0f,
        .yaw_output_sign = 1.0f,
    };

    return config;
}

int main(void)
{
    const ctrl_balance_controller_config_t config = make_config();
    const ctrl_balance_reference_t stopped_reference = {0};
    ctrl_balance_controller_t controller;
    ctrl_balance_controller_output_t output;
    balance_state_t state;

    memset(&state, 0, sizeof(state));
    state.valid = true;
    state.position_x_m = 1.0f;

    if (!ctrl_balance_controller_init(&controller, &config))
    {
        fail("controller initialization failed");
    }

    ctrl_balance_controller_reset(&controller, &state);
    state.position_x_m = 1.1f;
    state.velocity_x_mps = 0.2f;
    state.body_forward_mps = 0.2f;
    state.lean_rad = 0.2f;
    state.lean_rate_rps = 0.1f;

    if (!ctrl_balance_controller_update(&controller,
                                        &state,
                                        &stopped_reference,
                                        0.01f,
                                        &output))
    {
        fail("valid controller update failed");
    }

    expect_near(output.forward_position_error_m, 0.1f);
    expect_near(output.forward_integral_m_s, 0.001f);
    expect_near(output.forward_current_a, 0.3799f);

    memset(&state, 0, sizeof(state));
    state.valid = true;
    ctrl_balance_controller_reset(&controller, &state);
    state.lean_rad = 1.0f;
    (void)ctrl_balance_controller_update(&controller,
                                         &state,
                                         &stopped_reference,
                                         0.01f,
                                         &output);
    expect_near(output.forward_current_a, 1.0f);

    if (!output.saturated)
    {
        fail("current saturation was not reported");
    }

    ctrl_balance_controller_reset(&controller, &state);
    const ctrl_balance_reference_t moving_reference =
    {
        .forward_velocity_mps = 1.0f,
        .lateral_velocity_mps = 1.0f,
        .yaw_rate_rps = 1.0f,
    };
    state.lean_rad = 0.0f;
    (void)ctrl_balance_controller_update(&controller,
                                         &state,
                                         &moving_reference,
                                         0.1f,
                                         &output);

    expect_near(output.forward_velocity_reference_mps, 0.1f);
    expect_near(output.lateral_velocity_reference_mps, 0.2f);
    expect_near(output.yaw_rate_reference_rps, 0.3f);

    if ((output.forward_current_a <= 0.0f) ||
        (output.lateral_current_a <= 0.0f) ||
        (output.yaw_current_a <= 0.0f))
    {
        fail("positive motion reference produced wrong current sign");
    }

    return EXIT_SUCCESS;
}
