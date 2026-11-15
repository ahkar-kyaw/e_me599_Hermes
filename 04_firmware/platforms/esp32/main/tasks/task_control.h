#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include "app/app_actuator_test_types.h"
#include "control/ctrl_balance_types.h"

void task_control_start(void);
void task_control_get_snapshot(control_snapshot_t *snapshot);
void task_control_get_actuator_test_snapshot(
    app_actuator_test_snapshot_t *snapshot);

#endif
