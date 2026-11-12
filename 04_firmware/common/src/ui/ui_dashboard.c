#include "ui/ui_dashboard.h"

#include <stddef.h>
#include <stdio.h>

#define UI_RAD_TO_DEG 57.2957795f

#define UI_COLOR_BLACK  ((ui_color_t)0x0000u)
#define UI_COLOR_WHITE  ((ui_color_t)0xFFFFu)
#define UI_COLOR_MUTED  ((ui_color_t)0x7BEFu)
#define UI_COLOR_PANEL  ((ui_color_t)0x0821u)
#define UI_COLOR_CYAN   ((ui_color_t)0x07FFu)
#define UI_COLOR_GREEN  ((ui_color_t)0x07E0u)
#define UI_COLOR_AMBER  ((ui_color_t)0xFD20u)
#define UI_COLOR_RED    ((ui_color_t)0xF800u)

static void ui_dashboard_draw_header(
    ui_canvas_t *canvas,
    const app_operator_controls_state_t *controls,
    bool active_page);
static void ui_dashboard_render_ready(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model);
static void ui_dashboard_render_active(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model);
static bool ui_dashboard_motors_ready(
    const actuator_snapshot_t *snapshot);
static const char *ui_dashboard_balance_state(
    app_balance_safety_state_t state);
static ui_color_t ui_dashboard_balance_color(
    app_balance_safety_state_t state);
static const char *ui_dashboard_inhibit(uint32_t flags);

void ui_dashboard_render(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model)
{
    if ((canvas == NULL) || (model == NULL) ||
        (model->controls == NULL) || (model->rc_status == NULL) ||
        (model->imu_status == NULL) ||
        (model->actuator_snapshot == NULL) ||
        (model->control_snapshot == NULL) ||
        (model->balance_snapshot == NULL))
    {
        return;
    }

    const bool active_page = model->controls->link_usable &&
        model->controls->master_enabled &&
        model->controls->balance_enabled;

    ui_canvas_clear(canvas, UI_COLOR_BLACK);
    ui_dashboard_draw_header(canvas, model->controls, active_page);

    if (active_page)
    {
        ui_dashboard_render_active(canvas, model);
    }
    else
    {
        ui_dashboard_render_ready(canvas, model);
    }
}

static void ui_dashboard_draw_header(
    ui_canvas_t *canvas,
    const app_operator_controls_state_t *controls,
    bool active_page)
{
    char switches[20];

    ui_canvas_fill(canvas, 0u, 0u, canvas->width, 20u, UI_COLOR_PANEL);
    ui_canvas_draw_text(canvas,
                        4u,
                        6u,
                        active_page ? "ACTIVE" : "READY",
                        active_page ? UI_COLOR_GREEN : UI_COLOR_CYAN,
                        1u);

    (void)snprintf(switches,
                   sizeof(switches),
                   "SA %s SD %s",
                   controls->master_enabled ? "ON" : "OFF",
                   controls->balance_enabled ? "ON" : "OFF");
    const uint16_t width = ui_canvas_text_width(switches, 1u);
    const uint16_t x = (width < canvas->width) ?
        (uint16_t)(canvas->width - width - 4u) : 4u;
    ui_canvas_draw_text(canvas, x, 6u, switches, UI_COLOR_WHITE, 1u);
}

static void ui_dashboard_render_ready(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model)
{
    const control_snapshot_t *control = model->control_snapshot;
    const actuator_snapshot_t *actuator = model->actuator_snapshot;
    char line[32];

    (void)snprintf(line,
                   sizeof(line),
                   "LEAN %+.2f  RATE %+.2f",
                   (double)(control->state.lean_rad * UI_RAD_TO_DEG),
                   (double)control->state.lean_rate_rps);
    ui_canvas_draw_text(canvas, 4u, 27u, line, UI_COLOR_WHITE, 1u);

    (void)snprintf(line,
                   sizeof(line),
                   "TRIM %+.2f deg",
                   (double)(control->lean_trim_rad * UI_RAD_TO_DEG));
    ui_canvas_draw_text(canvas, 4u, 42u, line, UI_COLOR_WHITE, 1u);
    ui_canvas_draw_text(canvas, 104u, 42u, "SE ZERO", UI_COLOR_CYAN, 1u);

    (void)snprintf(line,
                   sizeof(line),
                   "RC %s  IMU %s  CAN %s",
                   model->rc_status->safe_for_control ? "OK" : "BAD",
                   model->imu_status->safe_for_balance ? "OK" : "BAD",
                   (actuator->bus_initialized &&
                    (actuator->bus_state == APP_ACTUATOR_BUS_ACTIVE)) ?
                       "OK" : "BAD");
    ui_canvas_draw_text(canvas,
                        4u,
                        57u,
                        line,
                        (model->rc_status->safe_for_control &&
                         model->imu_status->safe_for_balance &&
                         actuator->bus_initialized &&
                         (actuator->bus_state == APP_ACTUATOR_BUS_ACTIVE)) ?
                            UI_COLOR_GREEN : UI_COLOR_AMBER,
                        1u);

    (void)snprintf(line,
                   sizeof(line),
                   "MOTOR %c %c %c %c",
                   actuator->actuator[0].valid ? '1' : '-',
                   actuator->actuator[1].valid ? '2' : '-',
                   actuator->actuator[2].valid ? '3' : '-',
                   actuator->actuator[3].valid ? '4' : '-');
    ui_canvas_draw_text(canvas,
                        4u,
                        72u,
                        line,
                        ui_dashboard_motors_ready(actuator) ?
                            UI_COLOR_GREEN : UI_COLOR_AMBER,
                        1u);

    if (!model->controls->link_usable)
    {
        ui_canvas_draw_text(canvas, 4u, 88u,
                            "RC LINK LOST", UI_COLOR_RED, 1u);
    }
    else if (!model->controls->master_enabled)
    {
        ui_canvas_draw_text(canvas, 4u, 88u,
                            "SA ON TO ENABLE", UI_COLOR_MUTED, 1u);
    }
    else if (control->zero_lean_rejected_count > 0u)
    {
        ui_canvas_draw_text(canvas, 4u, 88u,
                            "ZERO REJECTED: HOLD STILL", UI_COLOR_RED, 1u);
    }
    else if (model->rc_status->safe_for_control &&
             model->imu_status->safe_for_balance &&
             control->state.valid &&
             actuator->bus_initialized &&
             (actuator->bus_state == APP_ACTUATOR_BUS_ACTIVE) &&
             ui_dashboard_motors_ready(actuator))
    {
        ui_canvas_draw_text(canvas, 4u, 88u,
                            "READY - SD ON TO BALANCE", UI_COLOR_GREEN, 1u);
    }
    else
    {
        ui_canvas_draw_text(canvas, 4u, 88u,
                            "NOT READY - CHECK ABOVE", UI_COLOR_AMBER, 1u);
    }

    ui_canvas_draw_text(canvas, 4u, 106u,
                        "SA MASTER   SD BALANCE", UI_COLOR_MUTED, 1u);
}

static void ui_dashboard_render_active(
    ui_canvas_t *canvas,
    const ui_dashboard_model_t *model)
{
    const control_snapshot_t *control = model->control_snapshot;
    const app_balance_safety_snapshot_t *balance =
        model->balance_snapshot;
    char line[36];

    (void)snprintf(line,
                   sizeof(line),
                   "BAL %s  INH %s",
                   ui_dashboard_balance_state(balance->state),
                   ui_dashboard_inhibit(balance->inhibit_flags));
    ui_canvas_draw_text(canvas,
                        4u,
                        27u,
                        line,
                        ui_dashboard_balance_color(balance->state),
                        1u);

    (void)snprintf(line,
                   sizeof(line),
                   "LEAN %+.2f  RATE %+.2f",
                   (double)(control->state.lean_rad * UI_RAD_TO_DEG),
                   (double)control->state.lean_rate_rps);
    ui_canvas_draw_text(canvas, 4u, 42u, line, UI_COLOR_WHITE, 1u);

    (void)snprintf(line,
                   sizeof(line),
                   "CMD F%+.2f L%+.2f Y%+.2f",
                   (double)control->forward_velocity_reference_mps,
                   (double)control->lateral_velocity_reference_mps,
                   (double)control->yaw_rate_reference_rps);
    ui_canvas_draw_text(canvas, 4u, 57u, line, UI_COLOR_WHITE, 1u);

    (void)snprintf(line,
                   sizeof(line),
                   "VEL F%+.2f L%+.2f Y%+.2f",
                   (double)control->state.body_forward_mps,
                   (double)control->state.body_left_mps,
                   (double)control->state.yaw_rate_rps);
    ui_canvas_draw_text(canvas,
                        4u,
                        72u,
                        line,
                        UI_COLOR_WHITE,
                        1u);

    (void)snprintf(line,
                   sizeof(line),
                   "OUT F%+.2f L%+.2f Y%+.2f",
                   (double)control->forward_current_a,
                   (double)control->lateral_current_a,
                   (double)control->yaw_current_a);
    ui_canvas_draw_text(canvas,
                        4u,
                        88u,
                        line,
                        control->balance_output_saturated ?
                            UI_COLOR_RED : UI_COLOR_WHITE,
                        1u);

    ui_canvas_draw_text(canvas, 4u, 106u,
                        "CH3 FWD CH1 LEFT CH4 YAW",
                        UI_COLOR_MUTED, 1u);
}

static bool ui_dashboard_motors_ready(
    const actuator_snapshot_t *snapshot)
{
    for (uint32_t index = 0u; index < APP_ACTUATOR_COUNT; index++)
    {
        const actuator_feedback_snapshot_t *feedback =
            &snapshot->actuator[index];

        if (!feedback->configured || !feedback->valid ||
            (feedback->fault_code != 0u))
        {
            return false;
        }
    }

    return true;
}

static const char *ui_dashboard_balance_state(
    app_balance_safety_state_t state)
{
    switch (state)
    {
        case APP_BALANCE_SAFETY_ARMED:
            return "LIVE";

        case APP_BALANCE_SAFETY_WAIT_SAFE:
            return "WAIT SAFE";

        case APP_BALANCE_SAFETY_STOPPING:
            return "STOPPING";

        case APP_BALANCE_SAFETY_LOCKOUT:
            return "LOCKOUT";

        case APP_BALANCE_SAFETY_DISABLED:
        default:
            return "OFF";
    }
}

static ui_color_t ui_dashboard_balance_color(
    app_balance_safety_state_t state)
{
    switch (state)
    {
        case APP_BALANCE_SAFETY_ARMED:
            return UI_COLOR_GREEN;

        case APP_BALANCE_SAFETY_WAIT_SAFE:
        case APP_BALANCE_SAFETY_STOPPING:
            return UI_COLOR_AMBER;

        case APP_BALANCE_SAFETY_LOCKOUT:
            return UI_COLOR_RED;

        case APP_BALANCE_SAFETY_DISABLED:
        default:
            return UI_COLOR_MUTED;
    }
}

static const char *ui_dashboard_inhibit(uint32_t flags)
{
    if (flags == APP_BALANCE_INHIBIT_NONE)
    {
        return "NONE";
    }

    if ((flags & APP_BALANCE_INHIBIT_REARM) != 0u)
    {
        return "SD OFF";
    }

    if ((flags & APP_BALANCE_INHIBIT_RC) != 0u)
    {
        return "RC";
    }

    if ((flags & APP_BALANCE_INHIBIT_IMU) != 0u)
    {
        return "IMU";
    }

    if ((flags & APP_BALANCE_INHIBIT_BUS) != 0u)
    {
        return "CAN";
    }

    if ((flags & APP_BALANCE_INHIBIT_FEEDBACK) != 0u)
    {
        return "FEEDBACK";
    }

    if ((flags & APP_BALANCE_INHIBIT_MOTOR_FAULT) != 0u)
    {
        return "MOTOR";
    }

    if ((flags & APP_BALANCE_INHIBIT_CURRENT) != 0u)
    {
        return "CURRENT";
    }

    if ((flags & APP_BALANCE_INHIBIT_STATE) != 0u)
    {
        return "STATE";
    }

    if ((flags & APP_BALANCE_INHIBIT_OPERATOR) != 0u)
    {
        return "STICKS";
    }

    if ((flags & APP_BALANCE_INHIBIT_REQUEST_STALE) != 0u)
    {
        return "REQUEST AGE";
    }

    return "REQUEST";
}
