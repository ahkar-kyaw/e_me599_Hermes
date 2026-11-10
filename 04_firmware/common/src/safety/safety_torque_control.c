#include "safety/safety_torque_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(APP_ACTUATOR_COUNT == CTRL_BALANCE_ACTUATOR_COUNT,
               "torque safety requires four control outputs");

static bool safety_torque_control_config_valid(
    const safety_torque_control_config_t *config);
static bool safety_torque_control_request_fresh(
    const app_torque_control_request_t *request,
    uint64_t now_us,
    uint64_t max_age_us);
static bool safety_torque_control_state_fresh(
    const control_snapshot_t *snapshot,
    uint64_t now_us,
    uint64_t max_age_us);
static bool safety_torque_control_stick_neutral(
    const safety_torque_control_config_t *config,
    uint16_t value);
static uint32_t safety_torque_control_inhibits(
    const safety_torque_control_t *safety,
    const app_torque_control_request_t *request,
    bool request_fresh,
    const control_snapshot_t *control_snapshot,
    bool state_fresh,
    const rc_snapshot_t *rc_snapshot,
    const safety_rc_status_t *rc_status,
    const safety_imu_status_t *imu_status,
    const actuator_snapshot_t *actuator_snapshot,
    bool stick_neutral);
static void safety_torque_control_begin_stop(
    safety_torque_control_t *safety,
    uint64_t now_us,
    bool require_request_cycle);

bool safety_torque_control_init(
    safety_torque_control_t *safety,
    const safety_torque_control_config_t *config)
{
    if ((safety == NULL) ||
        !safety_torque_control_config_valid(config))
    {
        return false;
    }

    memset(safety, 0, sizeof(*safety));
    safety->config = *config;
    safety->state = APP_TORQUE_CONTROL_DISABLED;
    safety->initialized = true;
    return true;
}

void safety_torque_control_update(
    safety_torque_control_t *safety,
    const app_torque_control_request_t *request,
    const control_snapshot_t *control_snapshot,
    const rc_snapshot_t *rc_snapshot,
    const safety_rc_status_t *rc_status,
    const safety_imu_status_t *imu_status,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us,
    app_torque_control_snapshot_t *snapshot)
{
    if ((safety == NULL) || !safety->initialized ||
        (request == NULL) || (control_snapshot == NULL) ||
        (rc_snapshot == NULL) || (rc_status == NULL) ||
        (imu_status == NULL) || (actuator_snapshot == NULL) ||
        (snapshot == NULL))
    {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_us = now_us;
    memcpy(snapshot->requested_current_a,
           control_snapshot->desired_current_a,
           sizeof(snapshot->requested_current_a));

    const bool request_fresh = safety_torque_control_request_fresh(
        request,
        now_us,
        safety->config.request_max_age_us);
    const bool state_fresh = safety_torque_control_state_fresh(
        control_snapshot,
        now_us,
        safety->config.state_max_age_us);
    const bool stick_neutral = safety_torque_control_stick_neutral(
        &safety->config,
        rc_snapshot->channel[safety->config.current_channel]);
    const uint32_t base_inhibits = safety_torque_control_inhibits(
        safety,
        request,
        request_fresh,
        control_snapshot,
        state_fresh,
        rc_snapshot,
        rc_status,
        imu_status,
        actuator_snapshot,
        stick_neutral);
    uint32_t hard_inhibits = base_inhibits &
        ~APP_TORQUE_INHIBIT_REARM;

    if (!request->stick_must_remain_neutral ||
        (safety->state != APP_TORQUE_CONTROL_ARMED))
    {
        hard_inhibits &= ~APP_TORQUE_INHIBIT_STICK;
    }

    if (!request_fresh || !request->enabled)
    {
        safety->request_cycle_required = false;

        if (safety->state == APP_TORQUE_CONTROL_ARMED)
        {
            safety_torque_control_begin_stop(safety, now_us, false);
        }
        else if ((safety->state == APP_TORQUE_CONTROL_STOPPING) &&
                 (now_us < safety->stop_until_us))
        {
            /* Keep transmitting zero current through the stop window. */
        }
        else
        {
            safety->state = APP_TORQUE_CONTROL_DISABLED;
        }
    }
    else if (hard_inhibits != APP_TORQUE_INHIBIT_NONE)
    {
        if (safety->state == APP_TORQUE_CONTROL_ARMED)
        {
            safety_torque_control_begin_stop(safety, now_us, true);
        }
        else if ((safety->state == APP_TORQUE_CONTROL_STOPPING) &&
                 (now_us < safety->stop_until_us))
        {
            /* Keep transmitting zero current through the stop window. */
        }
        else
        {
            safety->state = APP_TORQUE_CONTROL_WAIT_SAFE;
        }
    }
    else if (safety->state == APP_TORQUE_CONTROL_STOPPING)
    {
        if (now_us >= safety->stop_until_us)
        {
            safety->state = APP_TORQUE_CONTROL_WAIT_NEUTRAL;
        }
    }
    else if (safety->state == APP_TORQUE_CONTROL_ARMED)
    {
        /* Remain armed while every hard safety condition remains clear. */
    }
    else if (safety->request_cycle_required)
    {
        safety->state = APP_TORQUE_CONTROL_WAIT_NEUTRAL;
    }
    else if (!stick_neutral)
    {
        safety->state = APP_TORQUE_CONTROL_WAIT_NEUTRAL;
    }
    else
    {
        safety->state = APP_TORQUE_CONTROL_ARMED;
    }

    snapshot->state = safety->state;
    snapshot->inhibit_flags = base_inhibits;

    if (safety->request_cycle_required && request->enabled)
    {
        snapshot->inhibit_flags |= APP_TORQUE_INHIBIT_REARM;
    }

    if (safety->state == APP_TORQUE_CONTROL_ARMED)
    {
        memcpy(snapshot->approved_current_a,
               control_snapshot->desired_current_a,
               sizeof(snapshot->approved_current_a));
        snapshot->transmit_enabled = true;
        snapshot->motion_allowed = true;
        snapshot->inhibit_flags = APP_TORQUE_INHIBIT_NONE;
    }
    else if (safety->state == APP_TORQUE_CONTROL_STOPPING)
    {
        snapshot->transmit_enabled = true;
        snapshot->motion_allowed = false;
    }
}

static bool safety_torque_control_config_valid(
    const safety_torque_control_config_t *config)
{
    return (config != NULL) &&
           (config->request_max_age_us > 0u) &&
           (config->state_max_age_us > 0u) &&
           (config->stop_hold_us > 0u) &&
           (config->current_channel < APP_RC_CHANNEL_COUNT) &&
           (config->stick_neutral_low <= config->stick_neutral_high) &&
           isfinite(config->max_current_a) &&
           isfinite(config->arm_lean_limit_rad) &&
           isfinite(config->arm_lean_rate_limit_rps) &&
           isfinite(config->fall_lean_limit_rad) &&
           (config->max_current_a > 0.0f) &&
           (config->arm_lean_limit_rad > 0.0f) &&
           (config->arm_lean_rate_limit_rps > 0.0f) &&
           (config->fall_lean_limit_rad > config->arm_lean_limit_rad);
}

static bool safety_torque_control_request_fresh(
    const app_torque_control_request_t *request,
    uint64_t now_us,
    uint64_t max_age_us)
{
    return (request->timestamp_us != 0u) &&
           (now_us >= request->timestamp_us) &&
           ((now_us - request->timestamp_us) <= max_age_us);
}

static bool safety_torque_control_state_fresh(
    const control_snapshot_t *snapshot,
    uint64_t now_us,
    uint64_t max_age_us)
{
    return (snapshot->timestamp_us != 0u) &&
           (now_us >= snapshot->timestamp_us) &&
           ((now_us - snapshot->timestamp_us) <= max_age_us);
}

static bool safety_torque_control_stick_neutral(
    const safety_torque_control_config_t *config,
    uint16_t value)
{
    return (value >= config->stick_neutral_low) &&
           (value <= config->stick_neutral_high);
}

static uint32_t safety_torque_control_inhibits(
    const safety_torque_control_t *safety,
    const app_torque_control_request_t *request,
    bool request_fresh,
    const control_snapshot_t *control_snapshot,
    bool state_fresh,
    const rc_snapshot_t *rc_snapshot,
    const safety_rc_status_t *rc_status,
    const safety_imu_status_t *imu_status,
    const actuator_snapshot_t *actuator_snapshot,
    bool stick_neutral)
{
    uint32_t flags = APP_TORQUE_INHIBIT_NONE;

    if (!request->enabled)
    {
        flags |= APP_TORQUE_INHIBIT_REQUEST;
    }

    if (!request_fresh)
    {
        flags |= APP_TORQUE_INHIBIT_REQUEST_STALE;
    }

    if (!rc_snapshot->valid || !rc_status->safe_for_control)
    {
        flags |= APP_TORQUE_INHIBIT_RC;
    }

    if (!state_fresh || !control_snapshot->state.valid ||
        !control_snapshot->torque_command_valid ||
        (control_snapshot->input_fault_flags != CTRL_STATE_FAULT_NONE))
    {
        flags |= APP_TORQUE_INHIBIT_STATE;
    }

    if (!imu_status->safe_for_balance)
    {
        flags |= APP_TORQUE_INHIBIT_IMU;
    }

    const float lean_limit =
        (safety->state == APP_TORQUE_CONTROL_ARMED) ?
        safety->config.fall_lean_limit_rad :
        safety->config.arm_lean_limit_rad;

    if (fabsf(control_snapshot->state.lean_rad) > lean_limit ||
        ((safety->state != APP_TORQUE_CONTROL_ARMED) &&
         (fabsf(control_snapshot->state.lean_rate_rps) >
          safety->config.arm_lean_rate_limit_rps)))
    {
        flags |= APP_TORQUE_INHIBIT_STATE;
    }

    if (!actuator_snapshot->bus_initialized ||
        (actuator_snapshot->bus_state != APP_ACTUATOR_BUS_ACTIVE))
    {
        flags |= APP_TORQUE_INHIBIT_BUS;
    }

    for (uint32_t actuator = 0u;
         actuator < APP_ACTUATOR_COUNT;
         actuator++)
    {
        const actuator_feedback_snapshot_t *feedback =
            &actuator_snapshot->actuator[actuator];
        const float current_a =
            control_snapshot->desired_current_a[actuator];

        if (!feedback->configured || !feedback->valid)
        {
            flags |= APP_TORQUE_INHIBIT_FEEDBACK;
        }

        if (feedback->fault_code != 0u)
        {
            flags |= APP_TORQUE_INHIBIT_MOTOR_FAULT;
        }

        if (!isfinite(current_a) ||
            (fabsf(current_a) > safety->config.max_current_a))
        {
            flags |= APP_TORQUE_INHIBIT_CURRENT;
        }
    }

    if (!stick_neutral &&
        ((safety->state != APP_TORQUE_CONTROL_ARMED) ||
         request->stick_must_remain_neutral))
    {
        flags |= APP_TORQUE_INHIBIT_STICK;
    }

    if (safety->request_cycle_required)
    {
        flags |= APP_TORQUE_INHIBIT_REARM;
    }

    return flags;
}

static void safety_torque_control_begin_stop(
    safety_torque_control_t *safety,
    uint64_t now_us,
    bool require_request_cycle)
{
    safety->state = APP_TORQUE_CONTROL_STOPPING;
    safety->stop_until_us = now_us + safety->config.stop_hold_us;
    safety->request_cycle_required = require_request_cycle;
}
