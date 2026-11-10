#ifndef APP_SUPERVISOR_TYPES_H
#define APP_SUPERVISOR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_SYSTEM_MODE_SAFE_IDLE = 0,
    APP_SYSTEM_MODE_MANUAL_DRIVE,
    APP_SYSTEM_MODE_TORQUE_CONTROL
} app_system_mode_t;

typedef struct
{
    uint64_t timestamp_us;
    app_system_mode_t mode;
    uint8_t manual_drive_actuator_index;
} app_supervisor_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
