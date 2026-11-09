#include "task_motor.h"

#include "bsp_can_esp32.h"
#include "config_actuator.h"
#include "protocols/proto_cubemars_ak.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stddef.h>
#include <string.h>

#define TASK_MOTOR_STACK_BYTES 4096u
#define TASK_MOTOR_PRIORITY    (tskIDLE_PRIORITY + 3)

static const char *TAG = "task_motor";

static actuator_snapshot_t g_actuator_snapshot;
static portMUX_TYPE g_actuator_snapshot_lock =
    portMUX_INITIALIZER_UNLOCKED;
static app_manual_drive_snapshot_t g_approved_manual_command;
static portMUX_TYPE g_approved_manual_command_lock =
    portMUX_INITIALIZER_UNLOCKED;

typedef struct
{
    uint64_t next_tx_us;
    uint64_t stop_until_us;
    float velocity_erpm;
    uint8_t actuator_index;
    bool session_active;
    bool stopping;
} task_motor_tx_state_t;

static void task_motor_entry(void *argument);
static void task_motor_publish_snapshot(
    const actuator_snapshot_t *snapshot);
static void task_motor_initialize_snapshot(
    actuator_snapshot_t *snapshot);
static bool task_motor_config_is_valid(void);
static int task_motor_find_actuator(uint8_t motor_id);
static void task_motor_update_bus_status(
    actuator_snapshot_t *snapshot);
static app_actuator_bus_state_t task_motor_map_bus_state(
    bsp_can_esp32_bus_state_t state);
static void task_motor_update_freshness(
    actuator_snapshot_t *snapshot,
    uint64_t now_us);
static void task_motor_process_frame(
    actuator_snapshot_t *snapshot,
    const if_can_frame_t *frame,
    uint64_t timestamp_us);
static void task_motor_get_approved_manual_command(
    app_manual_drive_snapshot_t *command);
static void task_motor_update_command_tx(
    actuator_snapshot_t *snapshot,
    task_motor_tx_state_t *tx_state,
    uint64_t now_us);
static bool task_motor_manual_command_fresh(
    const app_manual_drive_snapshot_t *command,
    uint64_t now_us);
static void task_motor_send_velocity(
    actuator_snapshot_t *snapshot,
    uint8_t actuator_index,
    float velocity_erpm);

void task_motor_start(void)
{
    const BaseType_t ok = xTaskCreate(
        task_motor_entry,
        "task_motor",
        TASK_MOTOR_STACK_BYTES,
        NULL,
        TASK_MOTOR_PRIORITY,
        NULL);

    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create task");
    }
}

void task_motor_get_snapshot(actuator_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_actuator_snapshot_lock);
    *snapshot = g_actuator_snapshot;
    portEXIT_CRITICAL(&g_actuator_snapshot_lock);
}

void task_motor_set_approved_manual_command(
    const app_manual_drive_snapshot_t *command)
{
    if (command == NULL)
    {
        return;
    }

    portENTER_CRITICAL(&g_approved_manual_command_lock);
    g_approved_manual_command = *command;
    portEXIT_CRITICAL(&g_approved_manual_command_lock);
}

static void task_motor_entry(void *argument)
{
    (void)argument;

    actuator_snapshot_t snapshot;
    task_motor_tx_state_t tx_state;

    task_motor_initialize_snapshot(&snapshot);
    memset(&tx_state, 0, sizeof(tx_state));
    task_motor_publish_snapshot(&snapshot);

    if (!task_motor_config_is_valid())
    {
        ESP_LOGE(TAG, "invalid actuator configuration");
        vTaskDelete(NULL);
        return;
    }

    while (bsp_can_esp32_init(APP_ACTUATOR_CAN_BITRATE) !=
           BSP_CAN_ESP32_OK)
    {
        ESP_LOGE(TAG, "CAN init failed");
        vTaskDelay(pdMS_TO_TICKS(APP_ACTUATOR_CAN_RETRY_MS));
    }

    snapshot.bus_initialized = true;
    task_motor_update_bus_status(&snapshot);
    task_motor_publish_snapshot(&snapshot);

    ESP_LOGI(TAG,
             "CubeMars motor task started at %lu bit/s",
             (unsigned long)APP_ACTUATOR_CAN_BITRATE);

    for (;;)
    {
        if_can_frame_t frame;
        const bsp_can_esp32_result_t result =
            bsp_can_esp32_receive(
                &frame,
                APP_ACTUATOR_CAN_READ_TIMEOUT_MS);
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        if (result == BSP_CAN_ESP32_OK)
        {
            task_motor_process_frame(&snapshot, &frame, now_us);
        }

        task_motor_update_bus_status(&snapshot);

        if (snapshot.bus_state == APP_ACTUATOR_BUS_OFF)
        {
            bsp_can_esp32_recover_if_bus_off();
        }

        task_motor_update_freshness(&snapshot, now_us);
        task_motor_update_command_tx(&snapshot, &tx_state, now_us);
        task_motor_publish_snapshot(&snapshot);
    }
}

static void task_motor_publish_snapshot(
    const actuator_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_actuator_snapshot_lock);
    g_actuator_snapshot = *snapshot;
    portEXIT_CRITICAL(&g_actuator_snapshot_lock);
}

static void task_motor_initialize_snapshot(
    actuator_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->bus_state = APP_ACTUATOR_BUS_UNKNOWN;

    for (uint32_t i = 0u; i < APP_ACTUATOR_COUNT; i++)
    {
        snapshot->actuator[i].motor_id =
            APP_ACTUATOR_CONFIG[i].motor_id;
        snapshot->actuator[i].configured =
            APP_ACTUATOR_CONFIG[i].enabled;
    }
}

static bool task_motor_config_is_valid(void)
{
    for (uint32_t first = 0u;
         first < APP_ACTUATOR_COUNT;
         first++)
    {
        if ((APP_ACTUATOR_CONFIG[first].direction_sign != -1) &&
            (APP_ACTUATOR_CONFIG[first].direction_sign != 1))
        {
            return false;
        }

        if (!APP_ACTUATOR_CONFIG[first].enabled)
        {
            continue;
        }

        for (uint32_t second = first + 1u;
             second < APP_ACTUATOR_COUNT;
             second++)
        {
            if (APP_ACTUATOR_CONFIG[second].enabled &&
                (APP_ACTUATOR_CONFIG[first].motor_id ==
                 APP_ACTUATOR_CONFIG[second].motor_id))
            {
                return false;
            }
        }
    }

    return true;
}

static int task_motor_find_actuator(uint8_t motor_id)
{
    for (uint32_t i = 0u; i < APP_ACTUATOR_COUNT; i++)
    {
        if (APP_ACTUATOR_CONFIG[i].enabled &&
            (APP_ACTUATOR_CONFIG[i].motor_id == motor_id))
        {
            return (int)i;
        }
    }

    return -1;
}

static void task_motor_update_bus_status(
    actuator_snapshot_t *snapshot)
{
    bsp_can_esp32_status_t status;

    if (bsp_can_esp32_get_status(&status) != BSP_CAN_ESP32_OK)
    {
        snapshot->bus_initialized = false;
        snapshot->bus_state = APP_ACTUATOR_BUS_UNKNOWN;
        return;
    }

    snapshot->bus_initialized = status.initialized;
    snapshot->bus_state = task_motor_map_bus_state(status.state);
    snapshot->bus_error_count = status.bus_error_count;
    snapshot->rx_queue_overflow_count =
        status.rx_queue_overflow_count;
    snapshot->recovery_count = status.recovery_count;
}

static app_actuator_bus_state_t task_motor_map_bus_state(
    bsp_can_esp32_bus_state_t state)
{
    switch (state)
    {
        case BSP_CAN_ESP32_BUS_ACTIVE:
            return APP_ACTUATOR_BUS_ACTIVE;

        case BSP_CAN_ESP32_BUS_WARNING:
            return APP_ACTUATOR_BUS_WARNING;

        case BSP_CAN_ESP32_BUS_PASSIVE:
            return APP_ACTUATOR_BUS_PASSIVE;

        case BSP_CAN_ESP32_BUS_OFF:
            return APP_ACTUATOR_BUS_OFF;

        default:
            return APP_ACTUATOR_BUS_UNKNOWN;
    }
}

static void task_motor_update_freshness(
    actuator_snapshot_t *snapshot,
    uint64_t now_us)
{
    for (uint32_t i = 0u; i < APP_ACTUATOR_COUNT; i++)
    {
        actuator_feedback_snapshot_t *feedback =
            &snapshot->actuator[i];

        if (!feedback->configured ||
            (feedback->timestamp_us == 0u) ||
            (now_us < feedback->timestamp_us) ||
            ((now_us - feedback->timestamp_us) >
             APP_ACTUATOR_FEEDBACK_MAX_AGE_US) ||
            (snapshot->bus_state == APP_ACTUATOR_BUS_OFF))
        {
            feedback->valid = false;
        }
    }
}

static void task_motor_process_frame(
    actuator_snapshot_t *snapshot,
    const if_can_frame_t *frame,
    uint64_t timestamp_us)
{
    if (!frame->is_extended || frame->is_remote)
    {
        snapshot->ignored_frame_count++;
        return;
    }

    const uint8_t motor_id =
        proto_cubemars_ak_get_motor_id(frame);
    const int actuator_index =
        task_motor_find_actuator(motor_id);

    if (actuator_index < 0)
    {
        snapshot->unknown_motor_count++;
        return;
    }

    if (proto_cubemars_ak_get_function_id(frame) !=
        PROTO_CUBEMARS_AK_SERVO_FEEDBACK_FUNCTION_ID)
    {
        snapshot->ignored_frame_count++;
        return;
    }

    proto_cubemars_ak_feedback_t decoded;

    if (proto_cubemars_ak_decode_servo_feedback(
            frame,
            motor_id,
            &decoded) != PROTO_CUBEMARS_AK_OK)
    {
        snapshot->decode_error_count++;
        return;
    }

    actuator_feedback_snapshot_t *feedback =
        &snapshot->actuator[actuator_index];

    feedback->timestamp_us = timestamp_us;
    feedback->feedback_count++;
    feedback->position_deg = decoded.position_deg *
        (float)APP_ACTUATOR_CONFIG[actuator_index].direction_sign;
    feedback->velocity_erpm = decoded.velocity_erpm *
        (float)APP_ACTUATOR_CONFIG[actuator_index].direction_sign;
    feedback->current_a = decoded.current_a *
        (float)APP_ACTUATOR_CONFIG[actuator_index].direction_sign;
    feedback->temperature_c = decoded.temperature_c;
    feedback->fault_code = decoded.fault_code;
    feedback->valid = true;
}

static void task_motor_get_approved_manual_command(
    app_manual_drive_snapshot_t *command)
{
    portENTER_CRITICAL(&g_approved_manual_command_lock);
    *command = g_approved_manual_command;
    portEXIT_CRITICAL(&g_approved_manual_command_lock);
}

static void task_motor_update_command_tx(
    actuator_snapshot_t *snapshot,
    task_motor_tx_state_t *tx_state,
    uint64_t now_us)
{
    app_manual_drive_snapshot_t command;

    task_motor_get_approved_manual_command(&command);

    if (task_motor_manual_command_fresh(&command, now_us) &&
        command.transmit_enabled &&
        (command.actuator_index < APP_ACTUATOR_COUNT) &&
        APP_ACTUATOR_CONFIG[command.actuator_index].enabled)
    {
        tx_state->session_active = true;
        tx_state->stopping = !command.motion_allowed;
        tx_state->stop_until_us = 0u;
        tx_state->actuator_index = command.actuator_index;
        tx_state->velocity_erpm = command.motion_allowed ?
            command.velocity_erpm : 0.0f;
    }
    else if (tx_state->session_active && !tx_state->stopping)
    {
        tx_state->stopping = true;
        tx_state->stop_until_us =
            now_us + APP_MANUAL_DRIVE_STOP_HOLD_US;
        tx_state->velocity_erpm = 0.0f;
        tx_state->next_tx_us = 0u;
    }
    else if (tx_state->session_active && tx_state->stopping &&
             (tx_state->stop_until_us == 0u))
    {
        tx_state->stop_until_us =
            now_us + APP_MANUAL_DRIVE_STOP_HOLD_US;
    }

    if (tx_state->session_active && tx_state->stopping &&
        (tx_state->stop_until_us != 0u) &&
        (now_us >= tx_state->stop_until_us))
    {
        memset(tx_state, 0, sizeof(*tx_state));
    }

    snapshot->command_tx_active = tx_state->session_active;
    snapshot->command_actuator_index = tx_state->actuator_index;
    snapshot->last_command_velocity_erpm = tx_state->velocity_erpm;

    if (!tx_state->session_active ||
        ((tx_state->next_tx_us != 0u) &&
         (now_us < tx_state->next_tx_us)))
    {
        return;
    }

    task_motor_send_velocity(snapshot,
                             tx_state->actuator_index,
                             tx_state->velocity_erpm);
    tx_state->next_tx_us = now_us +
        ((uint64_t)APP_MANUAL_DRIVE_TASK_PERIOD_MS * 1000u);
}

static bool task_motor_manual_command_fresh(
    const app_manual_drive_snapshot_t *command,
    uint64_t now_us)
{
    return (command->timestamp_us != 0u) &&
           (now_us >= command->timestamp_us) &&
           ((now_us - command->timestamp_us) <=
            APP_MANUAL_DRIVE_COMMAND_MAX_AGE_US);
}

static void task_motor_send_velocity(
    actuator_snapshot_t *snapshot,
    uint8_t actuator_index,
    float velocity_erpm)
{
    if_can_frame_t frame;

    if ((actuator_index >= APP_ACTUATOR_COUNT) ||
        !APP_ACTUATOR_CONFIG[actuator_index].enabled ||
        (proto_cubemars_ak_encode_servo_velocity(
             APP_ACTUATOR_CONFIG[actuator_index].motor_id,
             velocity_erpm *
                 (float)APP_ACTUATOR_CONFIG[actuator_index].direction_sign,
             &frame) != PROTO_CUBEMARS_AK_OK))
    {
        snapshot->command_tx_error_count++;
        return;
    }

    if (bsp_can_esp32_transmit(
            &frame,
            APP_ACTUATOR_CAN_TX_TIMEOUT_MS) != BSP_CAN_ESP32_OK)
    {
        snapshot->command_tx_error_count++;
        return;
    }

    snapshot->command_tx_count++;
}
