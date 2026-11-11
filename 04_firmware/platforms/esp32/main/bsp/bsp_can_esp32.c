#include "bsp_can_esp32.h"

#if defined(BOARD_ESP32_NODEMCU_V1)
#include "board_esp32_nodemcu_v1.h"
#else
#error "No ESP32 board selected."
#endif

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <stddef.h>
#include <string.h>

#define BSP_CAN_ESP32_RX_QUEUE_LENGTH 32u
#define BSP_CAN_ESP32_TX_SLOT_COUNT    1u

static twai_node_handle_t g_can_node;
static QueueHandle_t g_can_rx_queue;
static QueueHandle_t g_can_tx_available_queue;
static twai_frame_t g_can_tx_frame;
static uint8_t g_can_tx_data[IF_CAN_CLASSIC_MAX_DATA_LENGTH];
static volatile uint32_t g_can_rx_queue_overflow_count;
static uint32_t g_can_recovery_count;
static bool g_can_initialized;
static bool g_can_recovery_requested;

static bool
bsp_can_esp32_tx_callback(twai_node_handle_t handle,
                          const twai_tx_done_event_data_t *event,
                          void *user_context);
static bool
bsp_can_esp32_rx_callback(twai_node_handle_t handle,
                          const twai_rx_done_event_data_t *event,
                          void *user_context);

static bsp_can_esp32_bus_state_t
bsp_can_esp32_map_state(twai_error_state_t state);

bsp_can_esp32_result_t bsp_can_esp32_init(uint32_t bitrate)
{
    if ((bitrate == 0u) || g_can_initialized)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    g_can_rx_queue = xQueueCreate(
        BSP_CAN_ESP32_RX_QUEUE_LENGTH,
        sizeof(if_can_frame_t));

    if (g_can_rx_queue == NULL)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    g_can_tx_available_queue = xQueueCreate(
        BSP_CAN_ESP32_TX_SLOT_COUNT,
        sizeof(uint8_t));

    if (g_can_tx_available_queue == NULL)
    {
        vQueueDelete(g_can_rx_queue);
        g_can_rx_queue = NULL;
        return BSP_CAN_ESP32_ERROR;
    }

    const uint8_t tx_slot = 0u;

    if (xQueueSend(g_can_tx_available_queue,
                   &tx_slot,
                   0u) != pdTRUE)
    {
        vQueueDelete(g_can_tx_available_queue);
        g_can_tx_available_queue = NULL;
        vQueueDelete(g_can_rx_queue);
        g_can_rx_queue = NULL;
        return BSP_CAN_ESP32_ERROR;
    }

    const twai_onchip_node_config_t node_config =
    {
        .io_cfg.tx = BOARD_CAN_TX_GPIO,
        .io_cfg.rx = BOARD_CAN_RX_GPIO,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate = bitrate,
        .fail_retry_cnt = 0,
        .tx_queue_depth = 1u,
        .flags.no_receive_rtr = true,
    };

    esp_err_t error = twai_new_node_onchip(&node_config, &g_can_node);

    if (error != ESP_OK)
    {
        vQueueDelete(g_can_tx_available_queue);
        g_can_tx_available_queue = NULL;
        vQueueDelete(g_can_rx_queue);
        g_can_rx_queue = NULL;
        return BSP_CAN_ESP32_ERROR;
    }

    const twai_event_callbacks_t callbacks =
    {
        .on_tx_done = bsp_can_esp32_tx_callback,
        .on_rx_done = bsp_can_esp32_rx_callback,
    };

    error = twai_node_register_event_callbacks(
        g_can_node,
        &callbacks,
        NULL);

    if (error != ESP_OK)
    {
        twai_node_delete(g_can_node);
        g_can_node = NULL;
        vQueueDelete(g_can_tx_available_queue);
        g_can_tx_available_queue = NULL;
        vQueueDelete(g_can_rx_queue);
        g_can_rx_queue = NULL;
        return BSP_CAN_ESP32_ERROR;
    }

    error = twai_node_enable(g_can_node);

    if (error != ESP_OK)
    {
        twai_node_delete(g_can_node);
        g_can_node = NULL;
        vQueueDelete(g_can_tx_available_queue);
        g_can_tx_available_queue = NULL;
        vQueueDelete(g_can_rx_queue);
        g_can_rx_queue = NULL;
        return BSP_CAN_ESP32_ERROR;
    }

    g_can_rx_queue_overflow_count = 0u;
    g_can_recovery_count = 0u;
    g_can_recovery_requested = false;
    g_can_initialized = true;

    return BSP_CAN_ESP32_OK;
}

bsp_can_esp32_result_t
bsp_can_esp32_receive(if_can_frame_t *frame, uint32_t timeout_ms)
{
    if (!g_can_initialized || (frame == NULL))
    {
        return BSP_CAN_ESP32_ERROR;
    }

    const BaseType_t received =
        xQueueReceive(g_can_rx_queue,
                      frame,
                      pdMS_TO_TICKS(timeout_ms));

    return (received == pdTRUE) ?
        BSP_CAN_ESP32_OK :
        BSP_CAN_ESP32_TIMEOUT;
}

bsp_can_esp32_result_t
bsp_can_esp32_transmit(const if_can_frame_t *frame, uint32_t timeout_ms)
{
    if (!g_can_initialized || (g_can_tx_available_queue == NULL) ||
        (frame == NULL) ||
        (frame->data_length > IF_CAN_CLASSIC_MAX_DATA_LENGTH))
    {
        return BSP_CAN_ESP32_ERROR;
    }

    uint8_t tx_slot;

    if (xQueueReceive(g_can_tx_available_queue,
                      &tx_slot,
                      pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return BSP_CAN_ESP32_TIMEOUT;
    }

    (void)tx_slot;
    memset(&g_can_tx_frame, 0, sizeof(g_can_tx_frame));
    memset(g_can_tx_data, 0, sizeof(g_can_tx_data));
    memcpy(g_can_tx_data, frame->data, frame->data_length);

    g_can_tx_frame.header.id = frame->id;
    g_can_tx_frame.header.dlc = frame->data_length;
    g_can_tx_frame.header.ide = frame->is_extended;
    g_can_tx_frame.header.rtr = frame->is_remote;
    g_can_tx_frame.buffer = g_can_tx_data;
    g_can_tx_frame.buffer_len = frame->data_length;

    const esp_err_t error = twai_node_transmit(
        g_can_node,
        &g_can_tx_frame,
        0);

    if (error == ESP_OK)
    {
        return BSP_CAN_ESP32_OK;
    }

    if (xQueueSend(g_can_tx_available_queue,
                   &tx_slot,
                   0u) != pdTRUE)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    return (error == ESP_ERR_TIMEOUT) ?
        BSP_CAN_ESP32_TIMEOUT : BSP_CAN_ESP32_ERROR;
}

bsp_can_esp32_result_t
bsp_can_esp32_get_status(bsp_can_esp32_status_t *status)
{
    if (!g_can_initialized || (status == NULL))
    {
        return BSP_CAN_ESP32_ERROR;
    }

    twai_node_status_t node_status;
    twai_node_record_t node_record;

    if (twai_node_get_info(g_can_node,
                           &node_status,
                           &node_record) != ESP_OK)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    memset(status, 0, sizeof(*status));
    status->state = bsp_can_esp32_map_state(node_status.state);
    status->bus_error_count = node_record.bus_err_num;
    status->rx_queue_overflow_count =
        g_can_rx_queue_overflow_count;
    status->recovery_count = g_can_recovery_count;
    status->initialized = true;

    if (node_status.state != TWAI_ERROR_BUS_OFF)
    {
        g_can_recovery_requested = false;
    }

    return BSP_CAN_ESP32_OK;
}

bsp_can_esp32_result_t bsp_can_esp32_recover_if_bus_off(void)
{
    if (!g_can_initialized)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    twai_node_status_t status;

    if (twai_node_get_info(g_can_node, &status, NULL) != ESP_OK)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    if ((status.state != TWAI_ERROR_BUS_OFF) ||
        g_can_recovery_requested)
    {
        return BSP_CAN_ESP32_OK;
    }

    if (twai_node_recover(g_can_node) != ESP_OK)
    {
        return BSP_CAN_ESP32_ERROR;
    }

    g_can_recovery_requested = true;
    g_can_recovery_count++;

    return BSP_CAN_ESP32_OK;
}

static bool
bsp_can_esp32_tx_callback(twai_node_handle_t handle,
                          const twai_tx_done_event_data_t *event,
                          void *user_context)
{
    (void)handle;
    (void)user_context;

    if ((event == NULL) ||
        (event->done_tx_frame != &g_can_tx_frame) ||
        (g_can_tx_available_queue == NULL))
    {
        return false;
    }

    const uint8_t tx_slot = 0u;
    BaseType_t higher_priority_task_woken = pdFALSE;

    (void)xQueueSendFromISR(g_can_tx_available_queue,
                            &tx_slot,
                            &higher_priority_task_woken);

    return higher_priority_task_woken == pdTRUE;
}

static bool
bsp_can_esp32_rx_callback(twai_node_handle_t handle,
                          const twai_rx_done_event_data_t *event,
                          void *user_context)
{
    (void)event;
    (void)user_context;

    uint8_t data[IF_CAN_CLASSIC_MAX_DATA_LENGTH];
    twai_frame_t twai_frame =
    {
        .buffer = data,
        .buffer_len = sizeof(data),
    };

    if (twai_node_receive_from_isr(handle, &twai_frame) != ESP_OK)
    {
        return false;
    }

    if_can_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.id = twai_frame.header.id;
    frame.data_length = (uint8_t)twai_frame.header.dlc;
    frame.is_extended = twai_frame.header.ide;
    frame.is_remote = twai_frame.header.rtr;

    if (frame.data_length > IF_CAN_CLASSIC_MAX_DATA_LENGTH)
    {
        return false;
    }

    memcpy(frame.data, data, frame.data_length);

    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xQueueSendFromISR(g_can_rx_queue,
                          &frame,
                          &higher_priority_task_woken) != pdTRUE)
    {
        g_can_rx_queue_overflow_count++;
    }

    return higher_priority_task_woken == pdTRUE;
}

static bsp_can_esp32_bus_state_t
bsp_can_esp32_map_state(twai_error_state_t state)
{
    switch (state)
    {
        case TWAI_ERROR_ACTIVE:
            return BSP_CAN_ESP32_BUS_ACTIVE;

        case TWAI_ERROR_WARNING:
            return BSP_CAN_ESP32_BUS_WARNING;

        case TWAI_ERROR_PASSIVE:
            return BSP_CAN_ESP32_BUS_PASSIVE;

        case TWAI_ERROR_BUS_OFF:
            return BSP_CAN_ESP32_BUS_OFF;

        default:
            return BSP_CAN_ESP32_BUS_UNKNOWN;
    }
}
