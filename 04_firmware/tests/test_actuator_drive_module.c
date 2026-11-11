#include "test_actuator_drive.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOLERANCE 1.0e-6f

static void expect_near(float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        (void)fprintf(stderr,
                      "expected %.6f, got %.6f\n",
                      (double)expected,
                      (double)actual);
        exit(EXIT_FAILURE);
    }
}

int main(void)
{
    const test_actuator_drive_config_t config =
    {
        .max_motor_current_a = 0.5f,
        .current_slew_rate_a_per_s = 1.0f,
    };
    test_actuator_drive_t test;
    test_actuator_drive_output_t output;

    if (!test_actuator_drive_init(&test, &config))
    {
        return EXIT_FAILURE;
    }

    if (!test_actuator_drive_update(&test,
                                    1.0f,
                                    0.0f,
                                    0.1f,
                                    &output))
    {
        return EXIT_FAILURE;
    }

    for (uint32_t actuator = 0u;
         actuator < APP_ACTUATOR_COUNT;
         actuator++)
    {
        expect_near(output.motor_current_a[actuator], 0.1f);
    }

    test_actuator_drive_reset(&test);
    (void)test_actuator_drive_update(&test,
                                     0.0f,
                                     1.0f,
                                     0.1f,
                                     &output);
    expect_near(output.motor_current_a[0], 0.1f);
    expect_near(output.motor_current_a[1], 0.1f);
    expect_near(output.motor_current_a[2], -0.1f);
    expect_near(output.motor_current_a[3], -0.1f);

    test_actuator_drive_reset(&test);
    (void)test_actuator_drive_update(&test,
                                     1.0f,
                                     1.0f,
                                     1.0f,
                                     &output);

    if (!output.saturated)
    {
        return EXIT_FAILURE;
    }

    expect_near(output.motor_current_a[0], 0.5f);
    expect_near(output.motor_current_a[1], 0.5f);
    expect_near(output.motor_current_a[2], 0.0f);
    expect_near(output.motor_current_a[3], 0.0f);

    return EXIT_SUCCESS;
}
