#ifndef APP_ACTUATOR_COMMAND_TYPES_H
#define APP_ACTUATOR_COMMAND_TYPES_H

#include "app/app_actuator_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_ACTUATOR_COMMAND_DISABLED = 0,
    APP_ACTUATOR_COMMAND_SERVO_VELOCITY,
    APP_ACTUATOR_COMMAND_SERVO_CURRENT
} app_actuator_command_mode_t;

typedef struct
{
    float value;
    app_actuator_command_mode_t mode;
    bool enabled;
} app_actuator_command_entry_t;

typedef struct
{
    uint64_t timestamp_us;
    app_actuator_command_entry_t actuator[APP_ACTUATOR_COUNT];
    bool valid;
} app_actuator_command_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
