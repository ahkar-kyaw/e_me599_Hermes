#ifndef APP_ACTUATOR_COMMAND_TYPES_H
#define APP_ACTUATOR_COMMAND_TYPES_H

#include "app/app_actuator_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float current_a;
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
