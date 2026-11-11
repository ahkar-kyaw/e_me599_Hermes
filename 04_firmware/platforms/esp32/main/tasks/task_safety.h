#ifndef TASK_SAFETY_H
#define TASK_SAFETY_H

#include "app/app_balance_safety_types.h"

void task_safety_start(void);
void task_safety_get_balance_snapshot(
    app_balance_safety_snapshot_t *snapshot);

#endif
