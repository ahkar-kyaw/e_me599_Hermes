#include "task_ui.h"

#include "bsp_display_spi_esp32.h"
#include "config_imu.h"
#include "config_rc.h"
#include "config_ui.h"
#include "drivers/drv_st7789.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_rc.h"
#include "task_safety.h"
#include "task_control.h"
#include "ui/ui_canvas.h"
#include "ui/ui_pages.h"
#include "ui/ui_rc_input.h"
#include "ui/ui_state.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdint.h>

#define TASK_UI_STACK_BYTES  6144u
#define TASK_UI_PRIORITY     (tskIDLE_PRIORITY + 1)

static const char *TAG = "task_ui";

static drv_st7789_t g_display;
static _Alignas(4)
    uint8_t g_display_framebuffer[DRV_ST7789_FRAMEBUFFER_SIZE];
static app_manual_drive_request_t g_manual_drive_request;
static portMUX_TYPE g_manual_drive_request_lock =
    portMUX_INITIALIZER_UNLOCKED;
static app_torque_control_request_t g_torque_control_request;
static portMUX_TYPE g_torque_control_request_lock =
    portMUX_INITIALIZER_UNLOCKED;

static void task_ui_entry(void *argument);
static void task_ui_draw_pixel(void *context,
                               uint16_t x,
                               uint16_t y,
                               ui_color_t color);
static void task_ui_fill_rect(void *context,
                              uint16_t x,
                              uint16_t y,
                              uint16_t width,
                              uint16_t height,
                              ui_color_t color);
static void task_ui_publish_manual_drive_request(
    const app_manual_drive_request_t *request);
static void task_ui_publish_torque_control_request(
    const app_torque_control_request_t *request);

void task_ui_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_ui_entry,
        "task_ui",
        TASK_UI_STACK_BYTES,
        NULL,
        TASK_UI_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_ui_get_manual_drive_request(
    app_manual_drive_request_t *request)
{
    if (request == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_manual_drive_request_lock);
    *request = g_manual_drive_request;
    portEXIT_CRITICAL(&g_manual_drive_request_lock);
}

void task_ui_get_torque_control_request(
    app_torque_control_request_t *request)
{
    if (request == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_torque_control_request_lock);
    *request = g_torque_control_request;
    portEXIT_CRITICAL(&g_torque_control_request_lock);
}

static void task_ui_entry(void *argument)
{
    (void)argument;

    bsp_display_spi_esp32_t display_io;
    drv_st7789_config_t display_config =
        drv_st7789_default_config();
    safety_rc_config_t rc_safety_config = safety_rc_default_config();
    safety_imu_config_t imu_safety_config = safety_imu_default_config();
    ui_rc_input_t rc_input;
    ui_state_t ui_state;
    ui_canvas_t canvas;

    const ui_rc_input_config_t rc_input_config =
    {
        .page_axis_channel =
            (uint8_t)(APP_UI_RC_PAGE_AXIS_CHANNEL - 1u),
        .vertical_axis_channel =
            (uint8_t)(APP_UI_RC_VERTICAL_AXIS_CHANNEL - 1u),
        .interact_channel =
            (uint8_t)(APP_UI_RC_INTERACT_CHANNEL - 1u),
        .enter_channel =
            (uint8_t)(APP_UI_RC_ENTER_CHANNEL - 1u),
        .input_enable_channel =
            (uint8_t)(APP_UI_RC_INPUT_ENABLE_CHANNEL - 1u),
        .channel_min = APP_RC_CHANNEL_MIN,
        .channel_max = APP_RC_CHANNEL_MAX,
        .axis_low_threshold = APP_UI_RC_AXIS_LOW_THRESHOLD,
        .axis_neutral_low = APP_UI_RC_AXIS_NEUTRAL_LOW,
        .axis_neutral_high = APP_UI_RC_AXIS_NEUTRAL_HIGH,
        .axis_high_threshold = APP_UI_RC_AXIS_HIGH_THRESHOLD,
        .action_off_threshold = APP_UI_RC_ACTION_OFF_THRESHOLD,
        .action_on_threshold = APP_UI_RC_ACTION_ON_THRESHOLD,
        .enable_off_threshold = APP_UI_RC_ENABLE_OFF_THRESHOLD,
        .enable_on_threshold = APP_UI_RC_ENABLE_ON_THRESHOLD,
        .page_right_high = (APP_UI_RC_PAGE_RIGHT_HIGH != 0),
        .vertical_up_high = (APP_UI_RC_VERTICAL_UP_HIGH != 0),
        .interact_active_high =
            (APP_UI_RC_INTERACT_ACTIVE_HIGH != 0),
        .enter_active_high = (APP_UI_RC_ENTER_ACTIVE_HIGH != 0),
        .input_enable_active_high =
            (APP_UI_RC_INPUT_ENABLE_ACTIVE_HIGH != 0),
    };

    display_config.orientation = APP_DISPLAY_ORIENTATION;

    rc_safety_config.max_age_us = APP_RC_MAX_AGE_US;
    rc_safety_config.channel_min = APP_RC_CHANNEL_MIN;
    rc_safety_config.channel_max = APP_RC_CHANNEL_MAX;
    rc_safety_config.channel_count = APP_RC_ACTIVE_CHANNEL_COUNT;

    imu_safety_config.max_age_us = APP_IMU_MAX_AGE_US;
    imu_safety_config.max_abs_roll_rad = APP_IMU_MAX_ABS_ROLL_RAD;
    imu_safety_config.max_abs_pitch_rad = APP_IMU_MAX_ABS_PITCH_RAD;
    imu_safety_config.min_accel_norm_mps2 =
        APP_IMU_MIN_ACCEL_NORM_MPS2;
    imu_safety_config.max_accel_norm_mps2 =
        APP_IMU_MAX_ACCEL_NORM_MPS2;

    if (!ui_rc_input_init(&rc_input, &rc_input_config))
    {
        ESP_LOGE(TAG, "invalid UI RC input config");
        vTaskDelete(NULL);
        return;
    }

    ui_state_init(&ui_state);

    while (bsp_display_spi_esp32_init(&display_io) !=
           IF_DISPLAY_IO_OK)
    {
        ESP_LOGE(TAG, "display SPI init failed");
        vTaskDelay(pdMS_TO_TICKS(APP_UI_RETRY_MS));
    }

    while (drv_st7789_init(&g_display,
                           &display_io.io,
                           &display_config,
                           g_display_framebuffer,
                           sizeof(g_display_framebuffer)) !=
           DRV_ST7789_OK)
    {
        ESP_LOGE(TAG, "ST7789 init failed");
        vTaskDelay(pdMS_TO_TICKS(APP_UI_RETRY_MS));
    }

    if (!ui_canvas_init(&canvas,
                        &g_display,
                        drv_st7789_width(&g_display),
                        drv_st7789_height(&g_display),
                        task_ui_draw_pixel,
                        task_ui_fill_rect))
    {
        ESP_LOGE(TAG, "invalid UI canvas config");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "UI started: page=CH%u vertical=CH%u interact=CH%u "
             "enter=CH%u enable=CH%u",
             (unsigned int)APP_UI_RC_PAGE_AXIS_CHANNEL,
             (unsigned int)APP_UI_RC_VERTICAL_AXIS_CHANNEL,
             (unsigned int)APP_UI_RC_INTERACT_CHANNEL,
             (unsigned int)APP_UI_RC_ENTER_CHANNEL,
             (unsigned int)APP_UI_RC_INPUT_ENABLE_CHANNEL);

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_render = last_wake;
    bool render_required = true;

    for (;;)
    {
        rc_snapshot_t rc_snapshot;
        imu_snapshot_t imu_snapshot;
        actuator_snapshot_t actuator_snapshot;
        app_manual_drive_snapshot_t manual_drive_snapshot;
        app_torque_control_snapshot_t torque_control_snapshot;
        control_snapshot_t control_snapshot;
        safety_rc_status_t rc_status;
        safety_imu_status_t imu_status;
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        task_rc_get_snapshot(&rc_snapshot);
        task_imu_get_snapshot(&imu_snapshot);
        task_motor_get_snapshot(&actuator_snapshot);
        task_safety_get_manual_drive_snapshot(&manual_drive_snapshot);
        task_safety_get_torque_control_snapshot(&torque_control_snapshot);
        task_control_get_snapshot(&control_snapshot);

        safety_rc_check(&rc_snapshot,
                        now_us,
                        &rc_safety_config,
                        &rc_status);
        (void)safety_imu_check(&imu_snapshot,
                               now_us,
                               &imu_safety_config,
                               &imu_status);

        const ui_event_flags_t events =
            ui_rc_input_update(&rc_input,
                               &rc_snapshot,
                               (rc_status.fault_flags &
                                (SAFETY_RC_FAULT_NO_DATA |
                                 SAFETY_RC_FAULT_STALE)) == 0u);

        if (ui_state_update(&ui_state, events))
        {
            render_required = true;

            ESP_LOGI(TAG,
                     "page=%u selection=%u mode=%s input=%s",
                     (unsigned int)(ui_state.page + 1u),
                     (unsigned int)(ui_state.selection + 1u),
                     (ui_state.mode == UI_MODE_INTERACT)
                         ? "interact"
                         : "browse",
                     ui_state.input_enabled ? "enabled" : "locked");
        }

        if ((events & UI_EVENT_ENTER) != 0u)
        {
            ESP_LOGI(TAG,
                     "enter requested on page=%u selection=%u",
                     (unsigned int)(ui_state.page + 1u),
                     (unsigned int)(ui_state.selection + 1u));
        }

        const app_manual_drive_request_t manual_drive_request =
        {
            .timestamp_us = now_us,
            .actuator_index = ui_state.selection,
            .enabled = ui_state.input_enabled &&
                       (ui_state.page == UI_PAGE_CAN) &&
                       (ui_state.mode == UI_MODE_INTERACT),
        };

        task_ui_publish_manual_drive_request(&manual_drive_request);

        const app_torque_control_request_t torque_control_request =
        {
            .timestamp_us = now_us,
            .enabled = ui_state.input_enabled &&
                       (ui_state.page == UI_PAGE_CONTROL) &&
                       (ui_state.mode == UI_MODE_INTERACT),
        };

        task_ui_publish_torque_control_request(&torque_control_request);

        const TickType_t now_tick = xTaskGetTickCount();

        if (render_required ||
            ((now_tick - last_render) >=
             pdMS_TO_TICKS(APP_UI_RENDER_PERIOD_MS)))
        {
            const ui_page_model_t model =
            {
                .rc_snapshot = &rc_snapshot,
                .rc_status = &rc_status,
                .imu_snapshot = &imu_snapshot,
                .imu_status = &imu_status,
                .actuator_snapshot = &actuator_snapshot,
                .manual_drive_snapshot = &manual_drive_snapshot,
                .control_snapshot = &control_snapshot,
                .torque_control_snapshot = &torque_control_snapshot,
            };

            ui_pages_render(&canvas, &ui_state, &model);

            if (drv_st7789_update(&g_display) != DRV_ST7789_OK)
            {
                ESP_LOGE(TAG, "ST7789 update failed");
            }

            last_render = now_tick;
            render_required = false;
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_UI_TASK_PERIOD_MS));
    }
}

static void task_ui_publish_manual_drive_request(
    const app_manual_drive_request_t *request)
{
    portENTER_CRITICAL(&g_manual_drive_request_lock);
    g_manual_drive_request = *request;
    portEXIT_CRITICAL(&g_manual_drive_request_lock);
}

static void task_ui_publish_torque_control_request(
    const app_torque_control_request_t *request)
{
    portENTER_CRITICAL(&g_torque_control_request_lock);
    g_torque_control_request = *request;
    portEXIT_CRITICAL(&g_torque_control_request_lock);
}

static void task_ui_draw_pixel(void *context,
                               uint16_t x,
                               uint16_t y,
                               ui_color_t color)
{
    drv_st7789_draw_pixel((drv_st7789_t *)context, x, y, color);
}

static void task_ui_fill_rect(void *context,
                              uint16_t x,
                              uint16_t y,
                              uint16_t width,
                              uint16_t height,
                              ui_color_t color)
{
    drv_st7789_fill_rect((drv_st7789_t *)context,
                         x,
                         y,
                         width,
                         height,
                         color);
}
