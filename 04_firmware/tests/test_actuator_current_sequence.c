#include "test_actuator_current.h"

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
    const test_actuator_current_config_t config =
    {
        .current_a = 0.5f,
        .prepare_ms = 1000u,
        .positive_ms = 500u,
        .neutral_ms = 1000u,
        .negative_ms = 500u,
    };
    test_actuator_current_t test;
    actuator_snapshot_t actuator;
    app_actuator_test_snapshot_t snapshot;

    memset(&actuator, 0, sizeof(actuator));

    if (!test_actuator_current_init(&test, &config))
    {
        fail("actuator test initialization failed");
    }

    test_actuator_current_update(&test,
                                 true,
                                 false,
                                 0u,
                                 &actuator,
                                 1000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_READY) || snapshot.active)
    {
        fail("enabled actuator test did not wait for start request");
    }

    test_actuator_current_update(&test,
                                 true,
                                 false,
                                 1u,
                                 &actuator,
                                 2000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_PREPARE) ||
        !snapshot.active || (snapshot.run_count != 1u) ||
        (snapshot.command_current_a != 0.0f))
    {
        fail("start request did not begin preparation stage");
    }

    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        actuator.actuator[index].velocity_erpm = (float)(index * 100u);
    }

    test_actuator_current_update(&test,
                                 true,
                                 true,
                                 1u,
                                 &actuator,
                                 1002000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_POSITIVE) ||
        (snapshot.command_current_a != 0.5f))
    {
        fail("preparation stage did not enter positive pulse");
    }

    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        actuator.actuator[index].velocity_erpm +=
            (float)((index + 1u) * 1000u);
    }

    test_actuator_current_update(&test,
                                 true,
                                 true,
                                 1u,
                                 &actuator,
                                 1502000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_NEUTRAL) ||
        (snapshot.positive_velocity_delta_erpm[3] != 4000.0f))
    {
        fail("positive velocity change was not captured");
    }

    test_actuator_current_update(&test,
                                 true,
                                 true,
                                 1u,
                                 &actuator,
                                 2502000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_NEGATIVE) ||
        (snapshot.command_current_a != -0.5f))
    {
        fail("neutral stage did not enter negative pulse");
    }

    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        actuator.actuator[index].velocity_erpm -=
            (float)((index + 1u) * 500u);
    }

    test_actuator_current_update(&test,
                                 true,
                                 true,
                                 1u,
                                 &actuator,
                                 3002000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_COMPLETE) || snapshot.active ||
        (snapshot.negative_velocity_delta_erpm[3] != -2000.0f))
    {
        fail("negative velocity change was not captured");
    }

    test_actuator_current_update(&test,
                                 true,
                                 false,
                                 2u,
                                 &actuator,
                                 3100000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_PREPARE) ||
        (snapshot.run_count != 2u) ||
        (snapshot.positive_velocity_delta_erpm[3] != 0.0f))
    {
        fail("repeated start did not reset and restart the test");
    }

    test_actuator_current_update(&test,
                                 true,
                                 true,
                                 2u,
                                 &actuator,
                                 4100000u,
                                 &snapshot);
    test_actuator_current_update(&test,
                                 true,
                                 false,
                                 2u,
                                 &actuator,
                                 4101000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_ABORTED) || snapshot.active)
    {
        fail("lost motion permission did not abort the running test");
    }

    test_actuator_current_update(&test,
                                 false,
                                 false,
                                 2u,
                                 &actuator,
                                 3200000u,
                                 &snapshot);

    if ((snapshot.state != APP_ACTUATOR_TEST_DISABLED) || snapshot.active)
    {
        fail("disabled test remained active");
    }

    return EXIT_SUCCESS;
}
