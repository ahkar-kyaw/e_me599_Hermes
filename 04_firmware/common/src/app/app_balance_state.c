#include "app/app_balance_state.h"

#include <stddef.h>
#include <string.h>

bool app_balance_state_imu_ready(
    const safety_imu_status_t *imu_status)
{
    const uint32_t data_fault_mask =
        SAFETY_IMU_FAULT_INVALID |
        SAFETY_IMU_FAULT_NOT_CALIBRATED |
        SAFETY_IMU_FAULT_ATTITUDE |
        SAFETY_IMU_FAULT_STALE |
        SAFETY_IMU_FAULT_TIME;

    return (imu_status != NULL) &&
        ((imu_status->fault_flags & data_fault_mask) == 0u);
}

bool app_balance_state_get_lean(
    const imu_snapshot_t *imu,
    const safety_imu_status_t *imu_status,
    ctrl_balance_lean_axis_t lean_axis,
    float *lean_rad,
    float *lean_rate_rps)
{
    if ((imu == NULL) || (imu_status == NULL) ||
        (lean_rad == NULL) || (lean_rate_rps == NULL) ||
        !app_balance_state_imu_ready(imu_status) ||
        !imu->attitude.valid)
    {
        return false;
    }

    switch (lean_axis)
    {
        case CTRL_BALANCE_LEAN_ROLL:
            *lean_rad = imu->attitude.roll_rad;
            *lean_rate_rps = imu->attitude.roll_rate_rps;
            return true;

        case CTRL_BALANCE_LEAN_PITCH:
            *lean_rad = imu->attitude.pitch_rad;
            *lean_rate_rps = imu->attitude.pitch_rate_rps;
            return true;

        default:
            return false;
    }
}

bool app_balance_state_compose(
    const imu_snapshot_t *imu,
    const safety_imu_status_t *imu_status,
    const est_collinear_mecanum_output_t *odometry,
    ctrl_balance_lean_axis_t lean_axis,
    uint64_t timestamp_us,
    balance_state_t *balance_state)
{
    if ((imu == NULL) || (imu_status == NULL) ||
        (odometry == NULL) || (balance_state == NULL))
    {
        return false;
    }

    memset(balance_state, 0, sizeof(*balance_state));
    balance_state->timestamp_us = timestamp_us;

    if (!odometry->valid ||
        !app_balance_state_get_lean(imu,
                                    imu_status,
                                    lean_axis,
                                    &balance_state->lean_rad,
                                    &balance_state->lean_rate_rps))
    {
        return false;
    }

    balance_state->position_x_m = odometry->position_x_m;
    balance_state->position_y_m = odometry->position_y_m;
    balance_state->yaw_rad = odometry->yaw_rad;
    balance_state->velocity_x_mps = odometry->velocity_x_mps;
    balance_state->velocity_y_mps = odometry->velocity_y_mps;
    balance_state->yaw_rate_rps = odometry->yaw_rate_rps;
    balance_state->valid = true;

    return true;
}
