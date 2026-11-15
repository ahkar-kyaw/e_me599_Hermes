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
    task_motor_start();
    task_control_start();
    task_rc_start();
    task_operator_start();
    task_supervisor_start();
    task_safety_start();
    task_ota_start();
    task_ui_start();
}
