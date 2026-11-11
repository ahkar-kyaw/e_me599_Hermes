#include "test_actuator_drive.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float test_actuator_drive_clamp(float value, float limit);
static float test_actuator_drive_slew(float current,
                                      float target,
                                      float rate,
                                      float dt_s);

bool test_actuator_drive_init(
    test_actuator_drive_t *test,
    const test_actuator_drive_config_t *config)
{
    if ((test == NULL) || (config == NULL) ||
        !isfinite(config->max_motor_current_a) ||
        !isfinite(config->current_slew_rate_a_per_s) ||
        (config->max_motor_current_a <= 0.0f) ||
        (config->current_slew_rate_a_per_s <= 0.0f))
    {
        return false;
    }

    memset(test, 0, sizeof(*test));
    test->config = *config;
    test->initialized = true;
    return true;
}

void test_actuator_drive_reset(test_actuator_drive_t *test)
{
    if ((test == NULL) || !test->initialized)
    {
        return;
    }

    test->forward_current_a = 0.0f;
    test->turn_current_a = 0.0f;
}

bool test_actuator_drive_update(
    test_actuator_drive_t *test,
    float forward_input,
    float turn_input,
    float dt_s,
    test_actuator_drive_output_t *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
    }

    if ((test == NULL) || !test->initialized || (output == NULL) ||
        !isfinite(forward_input) || !isfinite(turn_input) ||
        !isfinite(dt_s) || (dt_s <= 0.0f))
    {
        return false;
    }

    forward_input = test_actuator_drive_clamp(forward_input, 1.0f);
    turn_input = test_actuator_drive_clamp(turn_input, 1.0f);

    test->forward_current_a = test_actuator_drive_slew(
        test->forward_current_a,
        forward_input * test->config.max_motor_current_a,
        test->config.current_slew_rate_a_per_s,
        dt_s);
    test->turn_current_a = test_actuator_drive_slew(
        test->turn_current_a,
        turn_input * test->config.max_motor_current_a,
        test->config.current_slew_rate_a_per_s,
        dt_s);

    output->motor_current_a[0] =
        test->forward_current_a + test->turn_current_a;
    output->motor_current_a[1] =
        test->forward_current_a + test->turn_current_a;
    output->motor_current_a[2] =
        test->forward_current_a - test->turn_current_a;
    output->motor_current_a[3] =
        test->forward_current_a - test->turn_current_a;

    float peak_current_a = 0.0f;

    for (uint32_t actuator = 0u;
         actuator < APP_ACTUATOR_COUNT;
         actuator++)
    {
        peak_current_a = fmaxf(
            peak_current_a,
            fabsf(output->motor_current_a[actuator]));
    }

    if (peak_current_a > test->config.max_motor_current_a)
    {
        const float scale = test->config.max_motor_current_a /
            peak_current_a;

        for (uint32_t actuator = 0u;
             actuator < APP_ACTUATOR_COUNT;
             actuator++)
        {
            output->motor_current_a[actuator] *= scale;
        }

        output->saturated = true;
    }

    output->forward_current_a = test->forward_current_a;
    output->turn_current_a = test->turn_current_a;
    output->valid = true;
    return true;
}

static float test_actuator_drive_clamp(float value, float limit)
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

static float test_actuator_drive_slew(float current,
                                      float target,
                                      float rate,
                                      float dt_s)
{
    const float maximum_change = rate * dt_s;
    return current + test_actuator_drive_clamp(target - current,
                                                maximum_change);
}
