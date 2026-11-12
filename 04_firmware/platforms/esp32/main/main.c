#include "task_control.h"
#include "task_imu.h"
#include "task_motor.h"
#include "task_operator.h"
#include "task_ota.h"
#include "task_rc.h"
#include "task_safety.h"
#include "task_supervisor.h"
#include "task_ui.h"

#include "esp_log.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 firmware booting");

    task_imu_start();

#if APP_ENABLE_ACTUATORS
    task_motor_start();
#endif

#if APP_ENABLE_ACTUATORS && APP_ENABLE_CONTROL_STATE
    task_control_start();
#endif

#if APP_ENABLE_RC_RECEIVER
    task_rc_start();
    task_operator_start();
#endif

#if APP_ENABLE_ACTUATORS && APP_ENABLE_CONTROL_STATE && \
    APP_ENABLE_RC_RECEIVER
    task_supervisor_start();
    task_safety_start();
#endif

    task_ota_start();

#if APP_ENABLE_UI
    task_ui_start();
#endif
}
