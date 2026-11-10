#ifndef UI_PAGES_H
#define UI_PAGES_H

#include "app/app_actuator_types.h"
#include "app/app_imu_types.h"
#include "app/app_manual_drive_types.h"
#include "app/app_rc_types.h"
#include "app/app_torque_control_types.h"
#include "control/ctrl_balance_types.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "ui/ui_canvas.h"
#include "ui/ui_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    const rc_snapshot_t *rc_snapshot;
    const safety_rc_status_t *rc_status;
    const imu_snapshot_t *imu_snapshot;
    const safety_imu_status_t *imu_status;
    const actuator_snapshot_t *actuator_snapshot;
    const app_manual_drive_snapshot_t *manual_drive_snapshot;
    const control_snapshot_t *control_snapshot;
    const app_torque_control_snapshot_t *torque_control_snapshot;
} ui_page_model_t;

void ui_pages_render(ui_canvas_t *canvas,
                     const ui_state_t *state,
                     const ui_page_model_t *model);

#ifdef __cplusplus
}
#endif

#endif
