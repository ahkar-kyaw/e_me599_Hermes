#ifndef TASK_OPERATOR_H
#define TASK_OPERATOR_H

#include "app/app_balance_control_types.h"
#include "app/app_operator_controls.h"

void task_operator_start(void);

void task_operator_get_balance_control_request(
    app_balance_control_request_t *request);

void task_operator_get_controls_state(
    app_operator_controls_state_t *state);

#endif
