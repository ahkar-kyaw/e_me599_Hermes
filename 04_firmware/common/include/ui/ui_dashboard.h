#ifndef UI_DASHBOARD_H
#define UI_DASHBOARD_H

#include "app/app_actuator_types.h"
#include "app/app_balance_safety_types.h"
#include "app/app_operator_controls.h"
#include "control/ctrl_balance_types.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "ui/ui_canvas.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const app_operator_controls_state_t *controls;
    const safety_rc_status_t *rc_status;
    const safety_imu_status_t *imu_status;
    const actuator_snapshot_t *actuator_snapshot;
    const control_snapshot_t *control_snapshot;
    const app_balance_safety_snapshot_t *balance_snapshot;
} ui_dashboard_model_t;

void ui_dashboard_render(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model);

#ifdef __cplusplus
}
#endif

#endif
