#include "ui/ui_pages.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#define UI_RAD_TO_DEG 57.2957795f

#define UI_COLOR_BLACK       ((ui_color_t)0x0000u)
#define UI_COLOR_WHITE       ((ui_color_t)0xFFFFu)
#define UI_COLOR_MUTED       ((ui_color_t)0x7BEFu)
#define UI_COLOR_PANEL       ((ui_color_t)0x0821u)
#define UI_COLOR_PANEL_LIGHT ((ui_color_t)0x1082u)
#define UI_COLOR_CYAN        ((ui_color_t)0x07FFu)
#define UI_COLOR_GREEN       ((ui_color_t)0x07E0u)
#define UI_COLOR_AMBER       ((ui_color_t)0xFD20u)
#define UI_COLOR_RED         ((ui_color_t)0xF800u)
#define UI_COLOR_MAGENTA     ((ui_color_t)0xF81Fu)

#define UI_HEADER_HEIGHT 21u
#define UI_FOOTER_HEIGHT 14u
#define UI_LANDSCAPE_FOOTER_Y 106u

static void ui_pages_draw_header(ui_canvas_t *canvas,
                                 const ui_state_t *state);
static void ui_pages_draw_footer(ui_canvas_t *canvas,
                                 const ui_state_t *state,
                                 const ui_page_model_t *model);
static void ui_pages_render_status(ui_canvas_t *canvas,
                                   const ui_page_model_t *model);
static void ui_pages_render_rc(ui_canvas_t *canvas,
                               const ui_state_t *state,
                               const ui_page_model_t *model);
static void ui_pages_render_imu(ui_canvas_t *canvas,
                                const ui_state_t *state,
                                const ui_page_model_t *model);
static void ui_pages_render_control(ui_canvas_t *canvas,
                                    const ui_state_t *state,
                                    const ui_page_model_t *model);
static void ui_pages_render_can(ui_canvas_t *canvas,
                                const ui_state_t *state,
                                const ui_page_model_t *model);
static const char *ui_pages_rc_state(const safety_rc_status_t *status);
static const char *ui_pages_imu_state(const imu_snapshot_t *snapshot,
                                      const safety_imu_status_t *status);
static ui_color_t ui_pages_rc_color(const safety_rc_status_t *status);
static ui_color_t ui_pages_imu_color(
    const imu_snapshot_t *snapshot,
    const safety_imu_status_t *status);
static const char *ui_pages_can_bus_state(
    app_actuator_bus_state_t state);
static ui_color_t ui_pages_can_bus_color(
    app_actuator_bus_state_t state);
static const char *ui_pages_actuator_state(
    const actuator_snapshot_t *snapshot,
    const actuator_feedback_snapshot_t *feedback);
static ui_color_t ui_pages_actuator_color(
    const actuator_snapshot_t *snapshot,
    const actuator_feedback_snapshot_t *feedback);
static const char *ui_pages_manual_drive_state(
    const app_manual_drive_snapshot_t *snapshot);
static ui_color_t ui_pages_manual_drive_color(
    const app_manual_drive_snapshot_t *snapshot);
static const char *ui_pages_torque_control_state(
    const app_torque_control_snapshot_t *snapshot);
static ui_color_t ui_pages_torque_control_color(
    const app_torque_control_snapshot_t *snapshot);
static const char *ui_pages_torque_inhibit(uint32_t flags);
static uint32_t ui_pages_age_ms(uint64_t age_us);
static uint16_t ui_pages_footer_y(const ui_canvas_t *canvas);
static uint16_t ui_pages_content_offset(const ui_canvas_t *canvas);
static void ui_pages_format_age(char *text,
                                size_t text_size,
                                uint64_t age_us,
                                bool available);
static void ui_pages_draw_right_text(ui_canvas_t *canvas,
                                     uint16_t right,
                                     uint16_t y,
                                     const char *text,
                                     ui_color_t color,
                                     uint8_t scale);

void ui_pages_render(ui_canvas_t *canvas,
                     const ui_state_t *state,
                     const ui_page_model_t *model)
{
    if ((canvas == NULL) || (state == NULL) || (model == NULL) ||
        (model->rc_snapshot == NULL) || (model->rc_status == NULL) ||
        (model->imu_snapshot == NULL) || (model->imu_status == NULL) ||
        (model->actuator_snapshot == NULL) ||
        (model->manual_drive_snapshot == NULL) ||
        (model->control_snapshot == NULL) ||
        (model->torque_control_snapshot == NULL))
    {
        return;
    }

    ui_canvas_clear(canvas, UI_COLOR_BLACK);
    ui_pages_draw_header(canvas, state);

    switch (state->page)
    {
        case UI_PAGE_RC:
            ui_pages_render_rc(canvas, state, model);
            break;

        case UI_PAGE_IMU:
            ui_pages_render_imu(canvas, state, model);
            break;

        case UI_PAGE_CONTROL:
            ui_pages_render_control(canvas, state, model);
            break;

        case UI_PAGE_CAN:
            ui_pages_render_can(canvas, state, model);
            break;

        case UI_PAGE_STATUS:
        default:
            ui_pages_render_status(canvas, model);
            break;
    }

    ui_pages_draw_footer(canvas, state, model);
}

static void ui_pages_draw_header(ui_canvas_t *canvas,
                                 const ui_state_t *state)
{
    static const char *const page_name[UI_PAGE_COUNT] =
    {
        "STATUS",
        "CRSF",
        "IMU",
        "CONTROL",
        "CAN",
    };
    static const ui_color_t page_color[UI_PAGE_COUNT] =
    {
        UI_COLOR_CYAN,
        UI_COLOR_GREEN,
        UI_COLOR_MAGENTA,
        UI_COLOR_CYAN,
        UI_COLOR_AMBER,
    };
    const char *mode;
    ui_color_t mode_color;

    if (!state->input_enabled)
    {
        mode = "LOCK";
        mode_color = UI_COLOR_AMBER;
    }
    else if ((state->page == UI_PAGE_CAN) &&
             (state->mode == UI_MODE_INTERACT))
    {
        mode = "DRIVE";
        mode_color = UI_COLOR_AMBER;
    }
    else if ((state->page == UI_PAGE_CONTROL) &&
             (state->mode == UI_MODE_INTERACT))
    {
        mode = "TORQUE";
        mode_color = UI_COLOR_AMBER;
    }
    else if (state->mode == UI_MODE_INTERACT)
    {
        mode = "INTERACT";
        mode_color = UI_COLOR_GREEN;
    }
    else
    {
        mode = "BROWSE";
        mode_color = UI_COLOR_CYAN;
    }

    ui_canvas_fill(canvas,
                   0u,
                   0u,
                   canvas->width,
                   UI_HEADER_HEIGHT - 1u,
                   UI_COLOR_PANEL);
    ui_canvas_fill(canvas,
                   0u,
                   UI_HEADER_HEIGHT - 1u,
                   canvas->width,
                   1u,
                   page_color[state->page]);
    ui_canvas_draw_text(canvas,
                        4u,
                        3u,
                        page_name[state->page],
                        page_color[state->page],
                        2u);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 4u),
                             7u,
                             mode,
                             mode_color,
                             1u);
}

static void ui_pages_draw_footer(ui_canvas_t *canvas,
                                 const ui_state_t *state,
                                 const ui_page_model_t *model)
{
    char line[16];
    const uint16_t footer_y = ui_pages_footer_y(canvas);

    ui_canvas_fill(canvas,
                   0u,
                   footer_y,
                   canvas->width,
                   UI_FOOTER_HEIGHT,
                   UI_COLOR_PANEL);

    ui_canvas_draw_text(canvas,
                        4u,
                        footer_y + 4u,
                        "RC",
                        UI_COLOR_MUTED,
                        1u);
    ui_canvas_fill(canvas,
                   18u,
                   footer_y + 5u,
                   5u,
                   5u,
                   ui_pages_rc_color(model->rc_status));
    ui_canvas_draw_text(canvas,
                        27u,
                        footer_y + 4u,
                        "IMU",
                        UI_COLOR_MUTED,
                        1u);
    ui_canvas_fill(canvas,
                   47u,
                   footer_y + 5u,
                   5u,
                   5u,
                   ui_pages_imu_color(model->imu_snapshot,
                                      model->imu_status));
    ui_canvas_draw_text(canvas,
                        56u,
                        footer_y + 4u,
                        "CAN",
                        UI_COLOR_MUTED,
                        1u);
    ui_canvas_fill(canvas,
                   77u,
                   footer_y + 5u,
                   5u,
                   5u,
                   ui_pages_can_bus_color(
                       model->actuator_snapshot->bus_state));

    if ((state->mode == UI_MODE_INTERACT) &&
        (state->page != UI_PAGE_CAN) &&
        (ui_state_selection_count(state->page) > 1u) &&
        (canvas->width >= 140u))
    {
        (void)snprintf(line,
                       sizeof(line),
                       "%u/%u",
                       (unsigned int)(state->selection + 1u),
                       (unsigned int)ui_state_selection_count(
                           state->page));
        ui_pages_draw_right_text(canvas,
                                 (uint16_t)(canvas->width - 34u),
                                 footer_y + 4u,
                                 line,
                                 UI_COLOR_GREEN,
                                 1u);
    }

    (void)snprintf(line,
                   sizeof(line),
                   "%u/%u",
                   (unsigned int)(state->page + 1u),
                   (unsigned int)UI_PAGE_COUNT);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 4u),
                             footer_y + 4u,
                             line,
                             UI_COLOR_WHITE,
                             1u);
}

static void ui_pages_render_status(ui_canvas_t *canvas,
                                   const ui_page_model_t *model)
{
    char line[24];
    const uint16_t y_offset = ui_pages_content_offset(canvas);

    ui_canvas_fill(canvas, 4u, 25u + y_offset,
                   (uint16_t)(canvas->width - 8u), 18u,
                   UI_COLOR_PANEL_LIGHT);
    ui_canvas_draw_text(canvas, 9u, 31u + y_offset,
                        "RC", UI_COLOR_MUTED, 1u);
    ui_canvas_draw_text(canvas,
                        31u,
                        31u + y_offset,
                        ui_pages_rc_state(model->rc_status),
                        ui_pages_rc_color(model->rc_status),
                        1u);
    ui_pages_format_age(
        line,
        sizeof(line),
        model->rc_status->age_us,
        (model->rc_status->fault_flags &
         SAFETY_RC_FAULT_NO_DATA) == 0u);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 9u),
                             31u + y_offset,
                             line,
                             UI_COLOR_MUTED,
                             1u);

    ui_canvas_fill(canvas, 4u, 47u + y_offset,
                   (uint16_t)(canvas->width - 8u), 18u,
                   UI_COLOR_PANEL_LIGHT);
    ui_canvas_draw_text(canvas, 9u, 53u + y_offset,
                        "IMU", UI_COLOR_MUTED, 1u);
    ui_canvas_draw_text(canvas,
                        37u,
                        53u + y_offset,
                        ui_pages_imu_state(model->imu_snapshot,
                                           model->imu_status),
                        ui_pages_imu_color(model->imu_snapshot,
                                           model->imu_status),
                        1u);
    ui_pages_format_age(line,
                        sizeof(line),
                        model->imu_status->age_us,
                        model->imu_snapshot->valid);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 9u),
                             53u + y_offset,
                             line,
                             UI_COLOR_MUTED,
                             1u);

    (void)snprintf(line,
                   sizeof(line),
                   "R %+5.1f",
                   (double)(model->imu_snapshot->attitude.roll_rad *
                            UI_RAD_TO_DEG));
    ui_canvas_draw_text(canvas, 7u, 70u + y_offset,
                        line, UI_COLOR_WHITE, 2u);

    (void)snprintf(line,
                   sizeof(line),
                   "P %+5.1f",
                   (double)(model->imu_snapshot->attitude.pitch_rad *
                            UI_RAD_TO_DEG));
    ui_canvas_draw_text(canvas, 7u, 87u + y_offset,
                        line, UI_COLOR_WHITE, 2u);
}

static void ui_pages_render_rc(ui_canvas_t *canvas,
                               const ui_state_t *state,
                               const ui_page_model_t *model)
{
    char line[24];
    const uint32_t first_channel =
        (uint32_t)state->selection * 8u;
    const uint16_t y_offset = ui_pages_content_offset(canvas);
    const uint16_t right_column =
        (uint16_t)((canvas->width / 2u) + 6u);

    ui_canvas_draw_text(canvas,
                        4u,
                        25u + y_offset,
                        "LINK",
                        UI_COLOR_MUTED,
                        1u);
    ui_canvas_draw_text(canvas,
                        34u,
                        25u + y_offset,
                        ui_pages_rc_state(model->rc_status),
                        ui_pages_rc_color(model->rc_status),
                        1u);
    ui_pages_format_age(
        line,
        sizeof(line),
        model->rc_status->age_us,
        (model->rc_status->fault_flags &
         SAFETY_RC_FAULT_NO_DATA) == 0u);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 4u),
                             25u + y_offset,
                             line,
                             UI_COLOR_MUTED,
                             1u);

    for (uint32_t row = 0u; row < 4u; row++)
    {
        const uint32_t left_channel = first_channel + row;
        const uint32_t right_channel = first_channel + row + 4u;
        const uint16_t y =
            (uint16_t)(38u + y_offset + (row * 12u));

        (void)snprintf(line,
                       sizeof(line),
                       "%02lu %04u",
                       (unsigned long)(left_channel + 1u),
                       (unsigned int)model->rc_snapshot->channel[
                           left_channel]);
        ui_canvas_draw_text(canvas, 4u, y, line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "%02lu %04u",
                       (unsigned long)(right_channel + 1u),
                       (unsigned int)model->rc_snapshot->channel[
                           right_channel]);
        ui_canvas_draw_text(canvas, right_column, y,
                            line, UI_COLOR_WHITE, 1u);
    }

    (void)snprintf(line,
                   sizeof(line),
                   "CRC %lu  PARSE %lu",
                   (unsigned long)model->rc_snapshot->crc_error_count,
                   (unsigned long)model->rc_snapshot->parse_error_count);
    ui_canvas_draw_text(canvas, 4u, 91u + y_offset,
                        line, UI_COLOR_MUTED, 1u);
}

static void ui_pages_render_imu(ui_canvas_t *canvas,
                                const ui_state_t *state,
                                const ui_page_model_t *model)
{
    char line[24];
    const uint16_t y_offset = ui_pages_content_offset(canvas);

    if (state->selection == 0u)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "R %+5.1f",
                       (double)(model->imu_snapshot->attitude.roll_rad *
                                UI_RAD_TO_DEG));
        ui_canvas_draw_text(canvas, 7u, 25u + y_offset,
                            line, UI_COLOR_WHITE, 2u);

        (void)snprintf(line,
                       sizeof(line),
                       "P %+5.1f",
                       (double)(model->imu_snapshot->attitude.pitch_rad *
                                UI_RAD_TO_DEG));
        ui_canvas_draw_text(canvas, 7u, 45u + y_offset,
                            line, UI_COLOR_WHITE, 2u);

        (void)snprintf(line,
                       sizeof(line),
                       "Y %+5.1f",
                       (double)(model->imu_snapshot->attitude.yaw_rad *
                                UI_RAD_TO_DEG));
        ui_canvas_draw_text(canvas, 7u, 65u + y_offset,
                            line, UI_COLOR_WHITE, 2u);

        (void)snprintf(line,
                       sizeof(line),
                       "RATE %+6.3f  CAL %s",
                       (double)model->imu_snapshot->attitude.pitch_rate_rps,
                       model->imu_snapshot->calibrated ? "YES" : "NO");
        ui_canvas_draw_text(canvas, 4u, 92u + y_offset,
                            line, UI_COLOR_MUTED, 1u);
    }
    else
    {
        ui_canvas_draw_text(canvas,
                            4u,
                            25u + y_offset,
                            "ACCEL",
                            UI_COLOR_CYAN,
                            1u);
        ui_canvas_draw_text(canvas,
                            70u,
                            25u + y_offset,
                            "GYRO",
                            UI_COLOR_MAGENTA,
                            1u);

        for (uint32_t axis = 0u; axis < 3u; axis++)
        {
            static const char axis_name[] = {'X', 'Y', 'Z'};
            const uint16_t y =
                (uint16_t)(38u + y_offset + (axis * 13u));

            (void)snprintf(line,
                           sizeof(line),
                           "A%c %+5.2f",
                           axis_name[axis],
                           (double)model->imu_snapshot->data.accel_mps2[
                               axis]);
            ui_canvas_draw_text(canvas, 4u, y, line, UI_COLOR_WHITE, 1u);

            (void)snprintf(line,
                           sizeof(line),
                           "G%c %+5.2f",
                           axis_name[axis],
                           (double)model->imu_snapshot->data.gyro_rps[
                               axis]);
            ui_canvas_draw_text(canvas, 70u, y, line, UI_COLOR_WHITE, 1u);
        }

        (void)snprintf(line,
                       sizeof(line),
                       "TEMP %+4.1fC",
                       (double)model->imu_snapshot->data.temp_c);
        ui_canvas_draw_text(canvas, 4u, 80u + y_offset,
                            line, UI_COLOR_MUTED, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "NORM %5.2f",
                       (double)model->imu_status->accel_norm_mps2);
        ui_canvas_draw_text(canvas, 4u, 92u + y_offset,
                            line, UI_COLOR_MUTED, 1u);
    }
}

static void ui_pages_render_control(ui_canvas_t *canvas,
                                    const ui_state_t *state,
                                    const ui_page_model_t *model)
{
    const control_snapshot_t *control = model->control_snapshot;
    const app_torque_control_snapshot_t *torque =
        model->torque_control_snapshot;
    const uint16_t y_offset = ui_pages_content_offset(canvas);
    char line[32];

    if (state->selection == 0u)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "STATE %s  F%02lX",
                       control->state.valid ? "OK" : "INVALID",
                       (unsigned long)control->input_fault_flags);
        ui_canvas_draw_text(canvas,
                            4u,
                            25u + y_offset,
                            line,
                            control->state.valid ?
                                UI_COLOR_GREEN : UI_COLOR_RED,
                            1u);

        (void)snprintf(line,
                       sizeof(line),
                       "X%+.3f  Y%+.3f",
                       (double)control->state.position_x_m,
                       (double)control->state.position_y_m);
        ui_canvas_draw_text(canvas, 4u, 39u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "YAW%+.1f LEAN%+.1f",
                       (double)(control->state.yaw_rad * UI_RAD_TO_DEG),
                       (double)(control->state.lean_rad * UI_RAD_TO_DEG));
        ui_canvas_draw_text(canvas, 4u, 53u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "VX%+.3f VY%+.3f",
                       (double)control->state.velocity_x_mps,
                       (double)control->state.velocity_y_mps);
        ui_canvas_draw_text(canvas, 4u, 67u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "YR%+.2f LR%+.2f",
                       (double)control->state.yaw_rate_rps,
                       (double)control->state.lean_rate_rps);
        ui_canvas_draw_text(canvas, 4u, 81u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        ui_canvas_draw_text(canvas, 4u, 96u + y_offset,
                            "STATE 1/3", UI_COLOR_MUTED, 1u);
    }
    else if (state->selection == 1u)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "SNAP %s",
                       control->state.valid ? "VALID" : "INVALID");
        ui_canvas_draw_text(canvas,
                            4u,
                            25u + y_offset,
                            line,
                            control->state.valid ?
                                UI_COLOR_GREEN : UI_COLOR_RED,
                            1u);

        (void)snprintf(line,
                       sizeof(line),
                       "LOOP %lu",
                       (unsigned long)control->loop_count);
        ui_canvas_draw_text(canvas, 4u, 39u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "INVALID %lu  OVR %lu",
                       (unsigned long)control->invalid_input_count,
                       (unsigned long)control->overrun_count);
        ui_canvas_draw_text(canvas, 4u, 53u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "EXEC %luus MAX %luus",
                       (unsigned long)control->execution_time_us,
                       (unsigned long)control->max_execution_time_us);
        ui_canvas_draw_text(canvas, 4u, 67u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "FAULT 0x%02lX",
                       (unsigned long)control->input_fault_flags);
        ui_canvas_draw_text(canvas, 4u, 81u + y_offset,
                            line, UI_COLOR_MUTED, 1u);

        ui_canvas_draw_text(canvas, 4u, 96u + y_offset,
                            "SNAPSHOT 2/3", UI_COLOR_MUTED, 1u);
    }
    else
    {
        (void)snprintf(line,
                       sizeof(line),
                       "TORQUE %s",
                       ui_pages_torque_control_state(torque));
        ui_canvas_draw_text(canvas,
                            4u,
                            25u + y_offset,
                            line,
                            ui_pages_torque_control_color(torque),
                            1u);

        (void)snprintf(line,
                       sizeof(line),
                       "REQUEST %+.2f A",
                       (double)control->requested_current_a);
        ui_canvas_draw_text(canvas, 4u, 39u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "M1%+.2f M2%+.2f",
                       (double)torque->approved_current_a[0],
                       (double)torque->approved_current_a[1]);
        ui_canvas_draw_text(canvas, 4u, 53u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "M3%+.2f M4%+.2f",
                       (double)torque->approved_current_a[2],
                       (double)torque->approved_current_a[3]);
        ui_canvas_draw_text(canvas, 4u, 67u + y_offset,
                            line, UI_COLOR_WHITE, 1u);

        (void)snprintf(line,
                       sizeof(line),
                       "INH %-8s %03lX",
                       ui_pages_torque_inhibit(torque->inhibit_flags),
                       (unsigned long)torque->inhibit_flags);
        ui_canvas_draw_text(canvas, 4u, 81u + y_offset,
                            line, UI_COLOR_MUTED, 1u);

        ui_canvas_draw_text(canvas, 4u, 96u + y_offset,
                            "SC+SD ARM, THR CURRENT",
                            UI_COLOR_MUTED, 1u);
    }
}

static void ui_pages_render_can(ui_canvas_t *canvas,
                                const ui_state_t *state,
                                const ui_page_model_t *model)
{
    const actuator_snapshot_t *snapshot = model->actuator_snapshot;
    const uint32_t actuator_index =
        ((uint32_t)state->selection < APP_ACTUATOR_COUNT) ?
        (uint32_t)state->selection : 0u;
    const actuator_feedback_snapshot_t *feedback =
        &snapshot->actuator[actuator_index];
    char line[32];
    const uint16_t y_offset = ui_pages_content_offset(canvas);

    ui_canvas_draw_text(canvas, 4u, 25u + y_offset,
                        "BUS", UI_COLOR_MUTED, 1u);
    ui_canvas_draw_text(canvas,
                        28u,
                        25u + y_offset,
                        ui_pages_can_bus_state(snapshot->bus_state),
                        ui_pages_can_bus_color(snapshot->bus_state),
                        1u);
    ui_pages_draw_right_text(canvas,
                             (uint16_t)(canvas->width - 4u),
                             25u + y_offset,
                             "1 MBIT/S",
                             UI_COLOR_MUTED,
                             1u);

    (void)snprintf(line,
                   sizeof(line),
                   "TX ERR %lu",
                   (unsigned long)snapshot->command_tx_error_count);
    ui_canvas_draw_text(canvas, 4u, 37u + y_offset,
                        line, UI_COLOR_MUTED, 1u);

    ui_canvas_fill(canvas, 4u, 49u + y_offset,
                   (uint16_t)(canvas->width - 8u), 18u,
                   UI_COLOR_PANEL_LIGHT);
    (void)snprintf(line,
                   sizeof(line),
                   "M%lu  ID %u",
                   (unsigned long)(actuator_index + 1u),
                   (unsigned int)feedback->motor_id);
    ui_canvas_draw_text(canvas, 9u, 55u + y_offset,
                        line, UI_COLOR_WHITE, 1u);
    ui_pages_draw_right_text(
        canvas,
        (uint16_t)(canvas->width - 9u),
        55u + y_offset,
        ui_pages_actuator_state(snapshot, feedback),
        ui_pages_actuator_color(snapshot, feedback),
        1u);

    if (!feedback->configured)
    {
        ui_canvas_draw_text(canvas,
                            24u,
                            75u + y_offset,
                            "SLOT DISABLED",
                            UI_COLOR_MUTED,
                            1u);

        const app_manual_drive_snapshot_t *drive =
            model->manual_drive_snapshot;

        (void)snprintf(line,
                       sizeof(line),
                       "DRV %s",
                       ui_pages_manual_drive_state(drive));
        ui_canvas_draw_text(canvas,
                            4u,
                            96u + y_offset,
                            line,
                            ui_pages_manual_drive_color(drive),
                            1u);
        return;
    }

    (void)snprintf(line,
                   sizeof(line),
                   "P%+.1f V%+.0f",
                   (double)feedback->position_deg,
                   (double)feedback->velocity_erpm);
    ui_canvas_draw_text(canvas, 4u, 72u + y_offset,
                        line, UI_COLOR_WHITE, 1u);

    (void)snprintf(line,
                   sizeof(line),
                   "I%+.2fA T%dC F%u",
                   (double)feedback->current_a,
                   (int)feedback->temperature_c,
                   (unsigned int)feedback->fault_code);
    ui_canvas_draw_text(canvas,
                        4u,
                        84u + y_offset,
                        line,
                        (feedback->fault_code == 0u) ?
                            UI_COLOR_MUTED : UI_COLOR_RED,
                        1u);

    const app_manual_drive_snapshot_t *drive =
        model->manual_drive_snapshot;

    if (drive->state == APP_MANUAL_DRIVE_ARMED)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "DRV LIVE C%+.0f",
                       (double)drive->velocity_erpm);
    }
    else
    {
        (void)snprintf(line,
                       sizeof(line),
                       "DRV %s",
                       ui_pages_manual_drive_state(drive));
    }

    ui_canvas_draw_text(canvas,
                        4u,
                        96u + y_offset,
                        line,
                        ui_pages_manual_drive_color(drive),
                        1u);
}

static const char *ui_pages_rc_state(const safety_rc_status_t *status)
{
    if ((status->fault_flags & SAFETY_RC_FAULT_NO_DATA) != 0u)
    {
        return "WAIT";
    }

    if ((status->fault_flags & SAFETY_RC_FAULT_STALE) != 0u)
    {
        return "STALE";
    }

    if ((status->fault_flags & SAFETY_RC_FAULT_CHANNEL_RANGE) != 0u)
    {
        return "RANGE";
    }

    return "OK";
}

static const char *ui_pages_imu_state(const imu_snapshot_t *snapshot,
                                      const safety_imu_status_t *status)
{
    if (!snapshot->valid)
    {
        return snapshot->calibrated ? "WAIT" : "CAL";
    }

    if ((status->fault_flags & SAFETY_IMU_FAULT_STALE) != 0u)
    {
        return "STALE";
    }

    if (status->fault_flags != SAFETY_IMU_FAULT_NONE)
    {
        return "FAULT";
    }

    return "OK";
}

static ui_color_t ui_pages_rc_color(const safety_rc_status_t *status)
{
    if (status->fault_flags == SAFETY_RC_FAULT_NONE)
    {
        return UI_COLOR_GREEN;
    }

    if ((status->fault_flags & SAFETY_RC_FAULT_NO_DATA) != 0u)
    {
        return UI_COLOR_AMBER;
    }

    return UI_COLOR_RED;
}

static ui_color_t ui_pages_imu_color(
    const imu_snapshot_t *snapshot,
    const safety_imu_status_t *status)
{
    if (status->fault_flags == SAFETY_IMU_FAULT_NONE)
    {
        return snapshot->valid ? UI_COLOR_GREEN : UI_COLOR_AMBER;
    }

    return UI_COLOR_RED;
}

static const char *ui_pages_can_bus_state(
    app_actuator_bus_state_t state)
{
    switch (state)
    {
        case APP_ACTUATOR_BUS_ACTIVE:
            return "ACTIVE";

        case APP_ACTUATOR_BUS_WARNING:
            return "WARNING";

        case APP_ACTUATOR_BUS_PASSIVE:
            return "PASSIVE";

        case APP_ACTUATOR_BUS_OFF:
            return "OFF";

        default:
            return "WAIT";
    }
}

static ui_color_t ui_pages_can_bus_color(
    app_actuator_bus_state_t state)
{
    switch (state)
    {
        case APP_ACTUATOR_BUS_ACTIVE:
            return UI_COLOR_GREEN;

        case APP_ACTUATOR_BUS_WARNING:
            return UI_COLOR_AMBER;

        case APP_ACTUATOR_BUS_PASSIVE:
        case APP_ACTUATOR_BUS_OFF:
            return UI_COLOR_RED;

        default:
            return UI_COLOR_AMBER;
    }
}

static const char *ui_pages_actuator_state(
    const actuator_snapshot_t *snapshot,
    const actuator_feedback_snapshot_t *feedback)
{
    if (!feedback->configured)
    {
        return "DISABLED";
    }

    if (!snapshot->bus_initialized ||
        (feedback->timestamp_us == 0u))
    {
        return "WAIT";
    }

    if ((snapshot->bus_state == APP_ACTUATOR_BUS_OFF) ||
        (snapshot->bus_state == APP_ACTUATOR_BUS_PASSIVE))
    {
        return "BUS";
    }

    if (!feedback->valid)
    {
        return "STALE";
    }

    if (feedback->fault_code != 0u)
    {
        return "FAULT";
    }

    return "OK";
}

static ui_color_t ui_pages_actuator_color(
    const actuator_snapshot_t *snapshot,
    const actuator_feedback_snapshot_t *feedback)
{
    if (!feedback->configured ||
        !snapshot->bus_initialized ||
        (feedback->timestamp_us == 0u))
    {
        return UI_COLOR_AMBER;
    }

    if ((snapshot->bus_state == APP_ACTUATOR_BUS_OFF) ||
        (snapshot->bus_state == APP_ACTUATOR_BUS_PASSIVE) ||
        (feedback->fault_code != 0u))
    {
        return UI_COLOR_RED;
    }

    if (!feedback->valid)
    {
        return UI_COLOR_AMBER;
    }

    return UI_COLOR_GREEN;
}

static const char *ui_pages_manual_drive_state(
    const app_manual_drive_snapshot_t *snapshot)
{
    switch (snapshot->state)
    {
        case APP_MANUAL_DRIVE_WAIT_SAFE:
            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_RC) != 0u)
            {
                return "BLOCK RC";
            }

            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_BUS) != 0u)
            {
                return "BLOCK CAN";
            }

            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_MOTOR_FAULT) != 0u)
            {
                return "BLOCK FAULT";
            }

            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_FEEDBACK) != 0u)
            {
                return "BLOCK FB";
            }

            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_ACTUATOR) != 0u)
            {
                return "BLOCK SLOT";
            }

            return "BLOCK";

        case APP_MANUAL_DRIVE_WAIT_NEUTRAL:
            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_REARM) != 0u)
            {
                return "SD OFF-ON";
            }

            if ((snapshot->inhibit_flags &
                 APP_MANUAL_DRIVE_INHIBIT_STICK) != 0u)
            {
                return "CENTER THR";
            }

            return "READY";

        case APP_MANUAL_DRIVE_ARMED:
            return "LIVE";

        case APP_MANUAL_DRIVE_STOPPING:
            return "STOPPING";

        case APP_MANUAL_DRIVE_DISABLED:
        default:
            return "OFF SD=ENABLE";
    }
}

static ui_color_t ui_pages_manual_drive_color(
    const app_manual_drive_snapshot_t *snapshot)
{
    switch (snapshot->state)
    {
        case APP_MANUAL_DRIVE_ARMED:
            return UI_COLOR_GREEN;

        case APP_MANUAL_DRIVE_WAIT_SAFE:
        case APP_MANUAL_DRIVE_STOPPING:
            return UI_COLOR_RED;

        case APP_MANUAL_DRIVE_WAIT_NEUTRAL:
            return UI_COLOR_AMBER;

        case APP_MANUAL_DRIVE_DISABLED:
        default:
            return UI_COLOR_MUTED;
    }
}

static const char *ui_pages_torque_control_state(
    const app_torque_control_snapshot_t *snapshot)
{
    switch (snapshot->state)
    {
        case APP_TORQUE_CONTROL_WAIT_SAFE:
            return "BLOCKED";

        case APP_TORQUE_CONTROL_WAIT_NEUTRAL:
            if ((snapshot->inhibit_flags & APP_TORQUE_INHIBIT_REARM) != 0u)
            {
                return "SD OFF-ON";
            }

            return "CENTER THR";

        case APP_TORQUE_CONTROL_ARMED:
            return "LIVE";

        case APP_TORQUE_CONTROL_STOPPING:
            return "STOPPING";

        case APP_TORQUE_CONTROL_DISABLED:
        default:
            return "OFF";
    }
}

static ui_color_t ui_pages_torque_control_color(
    const app_torque_control_snapshot_t *snapshot)
{
    switch (snapshot->state)
    {
        case APP_TORQUE_CONTROL_ARMED:
            return UI_COLOR_GREEN;

        case APP_TORQUE_CONTROL_WAIT_SAFE:
        case APP_TORQUE_CONTROL_STOPPING:
            return UI_COLOR_RED;

        case APP_TORQUE_CONTROL_WAIT_NEUTRAL:
            return UI_COLOR_AMBER;

        case APP_TORQUE_CONTROL_DISABLED:
        default:
            return UI_COLOR_MUTED;
    }
}

static const char *ui_pages_torque_inhibit(uint32_t flags)
{
    if (flags == APP_TORQUE_INHIBIT_NONE)
    {
        return "NONE";
    }

    if ((flags & APP_TORQUE_INHIBIT_REARM) != 0u)
    {
        return "REARM";
    }

    if ((flags & APP_TORQUE_INHIBIT_RC) != 0u)
    {
        return "RC";
    }

    if ((flags & APP_TORQUE_INHIBIT_IMU) != 0u)
    {
        return "IMU";
    }

    if ((flags & APP_TORQUE_INHIBIT_BUS) != 0u)
    {
        return "CAN";
    }

    if ((flags & APP_TORQUE_INHIBIT_FEEDBACK) != 0u)
    {
        return "FEEDBACK";
    }

    if ((flags & APP_TORQUE_INHIBIT_MOTOR_FAULT) != 0u)
    {
        return "MOTOR";
    }

    if ((flags & APP_TORQUE_INHIBIT_CURRENT) != 0u)
    {
        return "CURRENT";
    }

    if ((flags & APP_TORQUE_INHIBIT_STATE) != 0u)
    {
        return "STATE";
    }

    if ((flags & APP_TORQUE_INHIBIT_STICK) != 0u)
    {
        return "STICK";
    }

    if ((flags & APP_TORQUE_INHIBIT_REQUEST_STALE) != 0u)
    {
        return "REQ AGE";
    }

    return "REQUEST";
}

static uint32_t ui_pages_age_ms(uint64_t age_us)
{
    const uint64_t age_ms = age_us / 1000u;

    return (age_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)age_ms;
}

static uint16_t ui_pages_footer_y(const ui_canvas_t *canvas)
{
    if ((canvas == NULL) || (canvas->height < UI_FOOTER_HEIGHT))
    {
        return 0u;
    }

    return (uint16_t)(canvas->height - UI_FOOTER_HEIGHT);
}

static uint16_t ui_pages_content_offset(const ui_canvas_t *canvas)
{
    const uint16_t footer_y = ui_pages_footer_y(canvas);

    return (footer_y > UI_LANDSCAPE_FOOTER_Y) ?
        (uint16_t)((footer_y - UI_LANDSCAPE_FOOTER_Y) / 2u) : 0u;
}

static void ui_pages_format_age(char *text,
                                size_t text_size,
                                uint64_t age_us,
                                bool available)
{
    if ((text == NULL) || (text_size == 0u))
    {
        return;
    }

    if (!available)
    {
        (void)snprintf(text, text_size, "--");
        return;
    }

    (void)snprintf(text,
                   text_size,
                   "%luMS",
                   (unsigned long)ui_pages_age_ms(age_us));
}

static void ui_pages_draw_right_text(ui_canvas_t *canvas,
                                     uint16_t right,
                                     uint16_t y,
                                     const char *text,
                                     ui_color_t color,
                                     uint8_t scale)
{
    const uint16_t width = ui_canvas_text_width(text, scale);
    const uint16_t x = (width < right) ? (uint16_t)(right - width) : 0u;

    ui_canvas_draw_text(canvas, x, y, text, color, scale);
}
