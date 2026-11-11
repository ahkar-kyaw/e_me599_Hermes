#include "safety/safety_balance_control.h"

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
    const safety_balance_control_config_t config =
    {
        .request_max_age_us = 100000u,
        .state_max_age_us = 20000u,
        .arm_dwell_us = 300000u,
        .stop_hold_us = 200000u,
        .max_current_a = 1.0f,
        .arm_lean_limit_rad = 0.2f,
        .arm_lean_rate_limit_rps = 1.0f,
        .fall_lean_limit_rad = 0.5f,
    };
    safety_balance_control_t safety;
    app_balance_safety_request_t request =
    {
        .timestamp_us = 1000u,
        .enabled = true,
    };
    control_snapshot_t control;
    rc_snapshot_t rc;
    safety_rc_status_t rc_status;
    safety_imu_status_t imu_status;
    actuator_snapshot_t actuator;
    app_balance_safety_snapshot_t snapshot;

    memset(&control, 0, sizeof(control));
    memset(&rc, 0, sizeof(rc));
    memset(&rc_status, 0, sizeof(rc_status));
    memset(&imu_status, 0, sizeof(imu_status));
    memset(&actuator, 0, sizeof(actuator));

    control.timestamp_us = 1000u;
    control.state.valid = true;
    control.balance_command_valid = true;
    rc.valid = true;
    rc_status.safe_for_control = true;
    imu_status.safe_for_balance = true;
    actuator.bus_initialized = true;
    actuator.bus_state = APP_ACTUATOR_BUS_ACTIVE;

    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        actuator.actuator[index].configured = true;
        actuator.actuator[index].valid = true;
    }

    if (!safety_balance_control_init(&safety, &config))
    {
        fail("balance safety initialization failed");
    }

    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  1000u,
                                  &snapshot);

    if ((snapshot.state != APP_BALANCE_SAFETY_WAIT_SAFE) ||
        snapshot.motion_allowed)
    {
        fail("safe request bypassed arming dwell");
    }

    request.timestamp_us = 301000u;
    control.timestamp_us = 301000u;
    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  301000u,
                                  &snapshot);

    if ((snapshot.state != APP_BALANCE_SAFETY_ARMED) ||
        !snapshot.motion_allowed)
    {
        fail("stable safe request did not arm after dwell");
    }

    actuator.actuator[1].valid = false;
    request.timestamp_us = 302000u;
    control.timestamp_us = 302000u;
    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  302000u,
                                  &snapshot);

    if ((snapshot.state != APP_BALANCE_SAFETY_STOPPING) ||
        snapshot.motion_allowed || !snapshot.transmit_enabled)
    {
        fail("feedback fault did not start zero-current stop");
    }

    actuator.actuator[1].valid = true;
    request.timestamp_us = 502001u;
    control.timestamp_us = 502001u;
    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  502001u,
                                  &snapshot);

    if ((snapshot.state != APP_BALANCE_SAFETY_WAIT_SAFE) ||
        snapshot.motion_allowed)
    {
        fail("cleared fault bypassed rearm dwell");
    }

    request.timestamp_us = 802001u;
    control.timestamp_us = 802001u;
    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  802001u,
                                  &snapshot);

    if ((snapshot.state != APP_BALANCE_SAFETY_ARMED) ||
        !snapshot.motion_allowed)
    {
        fail("cleared fault did not rearm after dwell");
    }

    request.enabled = false;
    request.timestamp_us = 803000u;
    control.timestamp_us = 803000u;
    safety_balance_control_update(&safety,
                                  &request,
                                  &control,
                                  &rc,
                                  &rc_status,
                                  &imu_status,
                                  &actuator,
                                  803000u,
                                  &snapshot);

    if (snapshot.state != APP_BALANCE_SAFETY_STOPPING)
    {
        fail("off request did not stop armed balance");
    }

    return EXIT_SUCCESS;
}
