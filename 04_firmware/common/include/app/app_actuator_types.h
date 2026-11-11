#ifndef APP_ACTUATOR_TYPES_H
#define APP_ACTUATOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ACTUATOR_COUNT 4u

typedef enum
{
    APP_ACTUATOR_BUS_UNKNOWN = 0,
    APP_ACTUATOR_BUS_ACTIVE,
    APP_ACTUATOR_BUS_WARNING,
    APP_ACTUATOR_BUS_PASSIVE,
    APP_ACTUATOR_BUS_OFF
} app_actuator_bus_state_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t feedback_count;
    float position_deg;
    float velocity_erpm;
    float current_a;
    int8_t temperature_c;
    uint8_t fault_code;
    uint8_t motor_id;
    bool configured;
    bool valid;
} actuator_feedback_snapshot_t;

typedef struct
{
    actuator_feedback_snapshot_t actuator[APP_ACTUATOR_COUNT];
    app_actuator_bus_state_t bus_state;
    uint32_t bus_error_count;
    uint32_t rx_queue_overflow_count;
    uint32_t decode_error_count;
    uint32_t unknown_motor_count;
    uint32_t ignored_frame_count;
    uint32_t recovery_count;
    uint32_t command_tx_count;
    uint32_t command_tx_error_count;
    bool bus_initialized;
    bool command_tx_active;
} actuator_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
