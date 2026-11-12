#ifndef APP_SUPERVISOR_TYPES_H
#define APP_SUPERVISOR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_SYSTEM_MODE_SAFE_IDLE = 0,
    APP_SYSTEM_MODE_BALANCE_CONTROL,
    APP_SYSTEM_MODE_UPDATE
} app_system_mode_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t zero_lean_request_count;
    float forward_input;
    float lateral_input;
    float yaw_input;
    app_system_mode_t mode;
} app_supervisor_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
