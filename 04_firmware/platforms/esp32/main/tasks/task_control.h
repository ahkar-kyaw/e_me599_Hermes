#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include "control/ctrl_balance_types.h"

void task_control_start(void);
void task_control_get_snapshot(control_snapshot_t *snapshot);

#endif
