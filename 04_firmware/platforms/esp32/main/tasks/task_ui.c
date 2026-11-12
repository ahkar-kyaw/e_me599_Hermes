#include "task_ui.h"

#include "bsp_display_spi_esp32.h"
#include "config_imu.h"
#include "config_rc.h"
#include "config_ui.h"
#include "drivers/drv_st7789.h"
#include "safety/safety_imu.h"
#include "safety/safety_rc.h"
#include "task_control.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_operator.h"
#include "task_ota.h"
#include "task_rc.h"
#include "task_safety.h"
#include "ui/ui_canvas.h"
#include "ui/ui_dashboard.h"

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

static void task_ui_entry(void *argument)
{
    (void)argument;

    bsp_display_spi_esp32_t display_io;
    drv_st7789_config_t display_config = drv_st7789_default_config();
    safety_rc_config_t rc_safety_config = safety_rc_default_config();
    safety_imu_config_t imu_safety_config = safety_imu_default_config();
    ui_canvas_t canvas;

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

    ESP_LOGI(TAG, "UI ready");

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_render = last_wake;

    for (;;)
    {
        rc_snapshot_t rc_snapshot;
        imu_snapshot_t imu_snapshot;
        actuator_snapshot_t actuator_snapshot;
        control_snapshot_t control_snapshot;
        app_balance_safety_snapshot_t balance_snapshot;
        app_ota_snapshot_t ota_snapshot;
        safety_rc_status_t rc_status;
        safety_imu_status_t imu_status;
        app_operator_controls_state_t controls_state;
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        task_rc_get_snapshot(&rc_snapshot);
        task_imu_get_snapshot(&imu_snapshot);
        task_motor_get_snapshot(&actuator_snapshot);
        task_control_get_snapshot(&control_snapshot);
        task_safety_get_balance_snapshot(&balance_snapshot);
        task_operator_get_controls_state(&controls_state);
        task_ota_get_snapshot(&ota_snapshot);

        safety_rc_check(&rc_snapshot,
                        now_us,
                        &rc_safety_config,
                        &rc_status);
        (void)safety_imu_check(&imu_snapshot,
                               now_us,
                               &imu_safety_config,
                               &imu_status);

        const TickType_t now_tick = xTaskGetTickCount();

        if ((now_tick - last_render) >=
            pdMS_TO_TICKS(APP_UI_RENDER_PERIOD_MS))
        {
            const ui_dashboard_model_t model =
            {
                .controls = &controls_state,
                .rc_status = &rc_status,
                .imu_status = &imu_status,
                .actuator_snapshot = &actuator_snapshot,
                .control_snapshot = &control_snapshot,
                .balance_snapshot = &balance_snapshot,
                .ota_snapshot = &ota_snapshot,
            };

            ui_dashboard_render(&canvas, &model);

            if (drv_st7789_update(&g_display) != DRV_ST7789_OK)
            {
                ESP_LOGE(TAG, "ST7789 update failed");
            }

            last_render = now_tick;
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(APP_UI_TASK_PERIOD_MS));
    }
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
