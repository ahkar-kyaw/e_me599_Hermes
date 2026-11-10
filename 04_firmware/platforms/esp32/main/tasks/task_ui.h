#ifndef TASK_UI_H
#define TASK_UI_H

#include "app/app_manual_drive_types.h"
#include "app/app_torque_control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void task_ui_start(void);
void task_ui_get_manual_drive_request(
    app_manual_drive_request_t *request);
void task_ui_get_torque_control_request(
    app_torque_control_request_t *request);

#ifdef __cplusplus
}
#endif

#endif
