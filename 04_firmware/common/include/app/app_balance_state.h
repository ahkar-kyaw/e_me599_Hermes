#ifndef APP_BALANCE_STATE_H
#define APP_BALANCE_STATE_H

#include "app/app_imu_types.h"
#include "control/ctrl_balance_types.h"
#include "estimation/est_collinear_mecanum.h"
#include "safety/safety_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

bool app_balance_state_imu_ready(
    const safety_imu_status_t *imu_status);

bool app_balance_state_compose(
    const imu_snapshot_t *imu,
    const safety_imu_status_t *imu_status,
    const est_collinear_mecanum_output_t *odometry,
    ctrl_balance_lean_axis_t lean_axis,
    uint64_t timestamp_us,
    balance_state_t *balance_state);

#ifdef __cplusplus
}
#endif

#endif
