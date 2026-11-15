#include "test_actuator_current.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool test_actuator_current_config_valid(
    const test_actuator_current_config_t *config);
static void test_actuator_current_start(
    test_actuator_current_t *test,
    uint64_t now_us);
static void test_actuator_current_set_state(
    test_actuator_current_t *test,
    app_actuator_test_state_t state,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us);
static void test_actuator_current_capture_delta(
    test_actuator_current_t *test,
    const actuator_snapshot_t *actuator_snapshot,
    float delta_erpm[APP_ACTUATOR_COUNT]);
static uint32_t test_actuator_current_stage_duration_ms(
    const test_actuator_current_t *test);
static float test_actuator_current_command(
    const test_actuator_current_t *test);

bool test_actuator_current_init(
    test_actuator_current_t *test,
    const test_actuator_current_config_t *config)
{
    if ((test == NULL) || !test_actuator_current_config_valid(config))
    {
        return false;
    }

    memset(test, 0, sizeof(*test));
    test->config = *config;
    test->state = APP_ACTUATOR_TEST_DISABLED;
    test->initialized = true;
    return true;
}

void test_actuator_current_update(
    test_actuator_current_t *test,
    bool enabled,
    bool motion_allowed,
    uint32_t start_request_count,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us,
    app_actuator_test_snapshot_t *snapshot)
{
    if (snapshot != NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
        snapshot->timestamp_us = now_us;
    }

    if ((test == NULL) || !test->initialized ||
        (actuator_snapshot == NULL) || (snapshot == NULL))
    {
        return;
    }

    if (!enabled)
    {
        test->state = APP_ACTUATOR_TEST_DISABLED;
        test->stage_started_us = 0u;
    }
    else
    {
        if (test->state == APP_ACTUATOR_TEST_DISABLED)
        {
            test->state = APP_ACTUATOR_TEST_READY;
            test->stage_started_us = now_us;
        }

        if (start_request_count != test->handled_start_request_count)
        {
            test->handled_start_request_count = start_request_count;
            test_actuator_current_start(test, now_us);
        }

        if (!motion_allowed &&
            ((test->state == APP_ACTUATOR_TEST_POSITIVE) ||
             (test->state == APP_ACTUATOR_TEST_NEUTRAL) ||
             (test->state == APP_ACTUATOR_TEST_NEGATIVE)))
        {
            test->state = APP_ACTUATOR_TEST_ABORTED;
            test->stage_started_us = now_us;
        }

        const uint32_t duration_ms =
            test_actuator_current_stage_duration_ms(test);

        if ((duration_ms > 0u) &&
            (now_us >= test->stage_started_us) &&
            ((now_us - test->stage_started_us) >=
             ((uint64_t)duration_ms * 1000u)) &&
            ((test->state != APP_ACTUATOR_TEST_PREPARE) ||
             motion_allowed))
        {
            switch (test->state)
            {
                case APP_ACTUATOR_TEST_PREPARE:
                    test_actuator_current_set_state(
                        test,
                        APP_ACTUATOR_TEST_POSITIVE,
                        actuator_snapshot,
                        now_us);
                    break;

                case APP_ACTUATOR_TEST_POSITIVE:
                    test_actuator_current_capture_delta(
                        test,
                        actuator_snapshot,
                        test->positive_velocity_delta_erpm);
                    test_actuator_current_set_state(
                        test,
                        APP_ACTUATOR_TEST_NEUTRAL,
                        actuator_snapshot,
                        now_us);
                    break;

                case APP_ACTUATOR_TEST_NEUTRAL:
                    test_actuator_current_set_state(
                        test,
                        APP_ACTUATOR_TEST_NEGATIVE,
                        actuator_snapshot,
                        now_us);
                    break;

                case APP_ACTUATOR_TEST_NEGATIVE:
                    test_actuator_current_capture_delta(
                        test,
                        actuator_snapshot,
                        test->negative_velocity_delta_erpm);
                    test_actuator_current_set_state(
                        test,
                        APP_ACTUATOR_TEST_COMPLETE,
                        actuator_snapshot,
                        now_us);
                    break;

                case APP_ACTUATOR_TEST_DISABLED:
                case APP_ACTUATOR_TEST_READY:
                case APP_ACTUATOR_TEST_COMPLETE:
                case APP_ACTUATOR_TEST_ABORTED:
                default:
                    break;
            }
        }
    }

    snapshot->run_count = test->run_count;
    snapshot->command_current_a = test_actuator_current_command(test);
    snapshot->state = test->state;
    snapshot->active =
        (test->state == APP_ACTUATOR_TEST_PREPARE) ||
        (test->state == APP_ACTUATOR_TEST_POSITIVE) ||
        (test->state == APP_ACTUATOR_TEST_NEUTRAL) ||
        (test->state == APP_ACTUATOR_TEST_NEGATIVE);

    memcpy(snapshot->positive_velocity_delta_erpm,
           test->positive_velocity_delta_erpm,
           sizeof(snapshot->positive_velocity_delta_erpm));
    memcpy(snapshot->negative_velocity_delta_erpm,
           test->negative_velocity_delta_erpm,
           sizeof(snapshot->negative_velocity_delta_erpm));

    const uint32_t duration_ms =
        test_actuator_current_stage_duration_ms(test);

    if ((duration_ms > 0u) && (now_us >= test->stage_started_us))
    {
        const uint64_t elapsed_ms =
            (now_us - test->stage_started_us) / 1000u;
        snapshot->remaining_ms = (elapsed_ms >= duration_ms) ?
            0u : (uint32_t)((uint64_t)duration_ms - elapsed_ms);
    }
}

static bool test_actuator_current_config_valid(
    const test_actuator_current_config_t *config)
{
    return (config != NULL) &&
           isfinite(config->current_a) &&
           (config->current_a > 0.0f) &&
           (config->prepare_ms > 0u) &&
           (config->positive_ms > 0u) &&
           (config->neutral_ms > 0u) &&
           (config->negative_ms > 0u);
}

static void test_actuator_current_start(
    test_actuator_current_t *test,
    uint64_t now_us)
{
    test->run_count++;
    test->state = APP_ACTUATOR_TEST_PREPARE;
    test->stage_started_us = now_us;
    memset(test->stage_start_velocity_erpm,
           0,
           sizeof(test->stage_start_velocity_erpm));
    memset(test->positive_velocity_delta_erpm,
           0,
           sizeof(test->positive_velocity_delta_erpm));
    memset(test->negative_velocity_delta_erpm,
           0,
           sizeof(test->negative_velocity_delta_erpm));
}

static void test_actuator_current_set_state(
    test_actuator_current_t *test,
    app_actuator_test_state_t state,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us)
{
    test->state = state;
    test->stage_started_us = now_us;

    if ((state == APP_ACTUATOR_TEST_POSITIVE) ||
        (state == APP_ACTUATOR_TEST_NEGATIVE))
    {
        for (uint32_t index = 0u;
             index < APP_ACTUATOR_COUNT;
             index++)
        {
            test->stage_start_velocity_erpm[index] =
                actuator_snapshot->actuator[index].velocity_erpm;
        }
    }
}

static void test_actuator_current_capture_delta(
    test_actuator_current_t *test,
    const actuator_snapshot_t *actuator_snapshot,
    float delta_erpm[APP_ACTUATOR_COUNT])
{
    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        delta_erpm[index] =
            actuator_snapshot->actuator[index].velocity_erpm -
            test->stage_start_velocity_erpm[index];
    }
}

static uint32_t test_actuator_current_stage_duration_ms(
    const test_actuator_current_t *test)
{
    switch (test->state)
    {
        case APP_ACTUATOR_TEST_PREPARE:
            return test->config.prepare_ms;

        case APP_ACTUATOR_TEST_POSITIVE:
            return test->config.positive_ms;

        case APP_ACTUATOR_TEST_NEUTRAL:
            return test->config.neutral_ms;

        case APP_ACTUATOR_TEST_NEGATIVE:
            return test->config.negative_ms;

        case APP_ACTUATOR_TEST_DISABLED:
        case APP_ACTUATOR_TEST_READY:
        case APP_ACTUATOR_TEST_COMPLETE:
        case APP_ACTUATOR_TEST_ABORTED:
        default:
            return 0u;
    }
}

static float test_actuator_current_command(
    const test_actuator_current_t *test)
{
    if (test->state == APP_ACTUATOR_TEST_POSITIVE)
    {
        return test->config.current_a;
    }

    if (test->state == APP_ACTUATOR_TEST_NEGATIVE)
    {
        return -test->config.current_a;
    }

    return 0.0f;
}
