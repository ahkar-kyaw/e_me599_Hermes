#include "safety/safety_torque_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_NOW_US 1000000u

static void fail(const char *message)
{
    (void)fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_state(const app_torque_control_snapshot_t *snapshot,
                         app_torque_control_state_t expected)
{
    if (snapshot->state != expected)
    {
        fail("unexpected torque-control state");
    }
}

int main(void)
{
    const safety_torque_control_config_t config =
    {
        .request_max_age_us = 100000u,
        .state_max_age_us = 20000u,
        .stop_hold_us = 200000u,
        .stick_neutral_low = 850u,
        .stick_neutral_high = 1134u,
        .max_current_a = 1.0f,
        .arm_lean_limit_rad = 0.1745329f,
        .arm_lean_rate_limit_rps = 1.3962634f,
        .fall_lean_limit_rad = 0.5235988f,
        .current_channel = 1u,
    };
    safety_torque_control_t safety;
    app_torque_control_request_t request =
    {
        .timestamp_us = TEST_NOW_US,
        .enabled = true,
        .stick_must_remain_neutral = false,
    };
    control_snapshot_t control;
    rc_snapshot_t rc;
    safety_rc_status_t rc_status =
    {
        .safe_for_control = true,
    };
    safety_imu_status_t imu_status =
    {
        .safe_for_balance = true,
    };
    actuator_snapshot_t actuators;
    app_torque_control_snapshot_t snapshot;

    memset(&control, 0, sizeof(control));
    memset(&rc, 0, sizeof(rc));
    memset(&actuators, 0, sizeof(actuators));

    control.timestamp_us = TEST_NOW_US;
    control.state.valid = true;
    control.torque_command_valid = true;
    rc.valid = true;
    rc.channel[config.current_channel] = 992u;
    actuators.bus_initialized = true;
    actuators.bus_state = APP_ACTUATOR_BUS_ACTIVE;

    for (uint32_t actuator = 0u;
         actuator < APP_ACTUATOR_COUNT;
         actuator++)
    {
        actuators.actuator[actuator].configured = true;
        actuators.actuator[actuator].valid = true;
        control.desired_current_a[actuator] = 0.25f;
    }

    if (!safety_torque_control_init(&safety, &config))
    {
        fail("torque-control initialization failed");
    }

    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 TEST_NOW_US,
                                 &snapshot);
    expect_state(&snapshot, APP_TORQUE_CONTROL_ARMED);

    if (!snapshot.transmit_enabled || !snapshot.motion_allowed ||
        (snapshot.approved_current_a[0] != 0.25f))
    {
        fail("safe current command was not approved");
    }

    rc_status.safe_for_control = false;
    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 TEST_NOW_US + 5000u,
                                 &snapshot);
    expect_state(&snapshot, APP_TORQUE_CONTROL_STOPPING);

    if (!snapshot.transmit_enabled || snapshot.motion_allowed ||
        (snapshot.approved_current_a[0] != 0.0f))
    {
        fail("fault stop did not command zero current");
    }

    rc_status.safe_for_control = true;
    request.timestamp_us = TEST_NOW_US + 210000u;
    control.timestamp_us = request.timestamp_us;
    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 request.timestamp_us,
                                 &snapshot);

    if ((snapshot.inhibit_flags & APP_TORQUE_INHIBIT_REARM) == 0u)
    {
        fail("fault recovery did not require a request cycle");
    }

    request.enabled = false;
    request.timestamp_us += 5000u;
    control.timestamp_us = request.timestamp_us;
    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 request.timestamp_us,
                                 &snapshot);

    request.enabled = true;
    request.timestamp_us += 5000u;
    control.timestamp_us = request.timestamp_us;
    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 request.timestamp_us,
                                 &snapshot);
    expect_state(&snapshot, APP_TORQUE_CONTROL_ARMED);

    request.stick_must_remain_neutral = true;
    rc.channel[config.current_channel] = 1400u;
    request.timestamp_us += 5000u;
    control.timestamp_us = request.timestamp_us;
    safety_torque_control_update(&safety,
                                 &request,
                                 &control,
                                 &rc,
                                 &rc_status,
                                 &imu_status,
                                 &actuators,
                                 request.timestamp_us,
                                 &snapshot);
    expect_state(&snapshot, APP_TORQUE_CONTROL_STOPPING);

    return EXIT_SUCCESS;
}
