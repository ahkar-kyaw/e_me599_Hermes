#ifndef APP_BALANCE_SAFETY_TYPES_H
#define APP_BALANCE_SAFETY_TYPES_H

#include "control/ctrl_balance_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_BALANCE_SAFETY_DISABLED = 0,
    APP_BALANCE_SAFETY_WAIT_SAFE,
    APP_BALANCE_SAFETY_ARMED,
    APP_BALANCE_SAFETY_STOPPING
} app_balance_safety_state_t;

typedef enum
{
    APP_BALANCE_INHIBIT_NONE          = 0u,
    APP_BALANCE_INHIBIT_REQUEST       = 1u << 0,
    APP_BALANCE_INHIBIT_REQUEST_STALE = 1u << 1,
    APP_BALANCE_INHIBIT_RC            = 1u << 2,
    APP_BALANCE_INHIBIT_IMU           = 1u << 3,
    APP_BALANCE_INHIBIT_STATE         = 1u << 4,
    APP_BALANCE_INHIBIT_BUS           = 1u << 5,
    APP_BALANCE_INHIBIT_FEEDBACK      = 1u << 6,
    APP_BALANCE_INHIBIT_MOTOR_FAULT   = 1u << 7,
    APP_BALANCE_INHIBIT_CURRENT       = 1u << 8
} app_balance_inhibit_t;

typedef struct
{
    uint64_t timestamp_us;
    bool enabled;
} app_balance_safety_request_t;

typedef struct
{
    uint64_t timestamp_us;
    float requested_current_a[CTRL_BALANCE_ACTUATOR_COUNT];
    float approved_current_a[CTRL_BALANCE_ACTUATOR_COUNT];
    uint32_t inhibit_flags;
    app_balance_safety_state_t state;
    bool transmit_enabled;
    bool motion_allowed;
} app_balance_safety_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
