#ifndef TASK_SAFETY_H
#define TASK_SAFETY_H

#include "app/app_manual_drive_types.h"
#include "app/app_torque_control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void task_safety_start(void);
void task_safety_get_manual_drive_snapshot(
    app_manual_drive_snapshot_t *snapshot);
void task_safety_get_torque_control_snapshot(
    app_torque_control_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
