#ifndef SAFETY_BALANCE_CONTROL_H
#define SAFETY_BALANCE_CONTROL_H

#include "app/app_actuator_types.h"
#include "app/app_balance_safety_types.h"
#include "app/app_rc_types.h"
#include "control/ctrl_balance_types.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint64_t request_max_age_us;
    uint64_t state_max_age_us;
    uint64_t arm_dwell_us;
    uint64_t stop_hold_us;
    float max_current_a;
    float arm_lean_limit_rad;
    float arm_lean_rate_limit_rps;
    float fall_lean_limit_rad;
} safety_balance_control_config_t;

typedef struct
{
    safety_balance_control_config_t config;
    app_balance_safety_state_t state;
    uint64_t safe_since_us;
    uint64_t stop_until_us;
    bool rearm_required;
    bool initialized;
} safety_balance_control_t;

bool safety_balance_control_init(
    safety_balance_control_t *safety,
    const safety_balance_control_config_t *config);

void safety_balance_control_update(
    safety_balance_control_t *safety,
    const app_balance_safety_request_t *request,
    const control_snapshot_t *control_snapshot,
    const rc_snapshot_t *rc_snapshot,
    const safety_rc_status_t *rc_status,
    const safety_imu_status_t *imu_status,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us,
    app_balance_safety_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
