#include "control/ctrl_torque_input.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOLERANCE 1.0e-5f

static void expect_near(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        (void)fprintf(stderr,
                      "%s: expected %.6f, got %.6f\n",
                      name,
                      (double)expected,
                      (double)actual);
        exit(EXIT_FAILURE);
    }
}

int main(void)
{
    const ctrl_torque_input_config_t config =
    {
        .channel_min = 172u,
        .channel_max = 1811u,
        .neutral_low = 850u,
        .neutral_high = 1134u,
        .max_current_a = 1.0f,
        .slew_rate_a_per_s = 4.0f,
        .positive_high = true,
    };
    ctrl_torque_input_t input;
    float current_a = 0.0f;

    if (!ctrl_torque_input_init(&input, &config))
    {
        return EXIT_FAILURE;
    }

    if (!ctrl_torque_input_update(&input, 992u, 0.005f, &current_a))
    {
        return EXIT_FAILURE;
    }
    expect_near("neutral current", current_a, 0.0f);

    if (!ctrl_torque_input_update(&input, 1811u, 0.005f, &current_a))
    {
        return EXIT_FAILURE;
    }
    expect_near("positive slew", current_a, 0.02f);

    for (uint32_t step = 0u; step < 100u; step++)
    {
        (void)ctrl_torque_input_update(&input,
                                       1811u,
                                       0.005f,
                                       &current_a);
    }
    expect_near("positive limit", current_a, 1.0f);

    ctrl_torque_input_reset(&input);
    (void)ctrl_torque_input_update(&input, 172u, 0.005f, &current_a);
    expect_near("negative slew", current_a, -0.02f);

    return EXIT_SUCCESS;
}
