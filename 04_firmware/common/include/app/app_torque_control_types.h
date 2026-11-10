#ifndef APP_TORQUE_CONTROL_TYPES_H
#define APP_TORQUE_CONTROL_TYPES_H

#include "app/app_actuator_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_TORQUE_CONTROL_DISABLED = 0,
    APP_TORQUE_CONTROL_WAIT_SAFE,
    APP_TORQUE_CONTROL_WAIT_NEUTRAL,
    APP_TORQUE_CONTROL_ARMED,
    APP_TORQUE_CONTROL_STOPPING
} app_torque_control_state_t;

typedef enum
{
    APP_TORQUE_INHIBIT_NONE           = 0u,
    APP_TORQUE_INHIBIT_REQUEST        = 1u << 0,
    APP_TORQUE_INHIBIT_REQUEST_STALE  = 1u << 1,
    APP_TORQUE_INHIBIT_RC             = 1u << 2,
    APP_TORQUE_INHIBIT_STATE          = 1u << 3,
    APP_TORQUE_INHIBIT_IMU            = 1u << 4,
    APP_TORQUE_INHIBIT_BUS            = 1u << 5,
    APP_TORQUE_INHIBIT_FEEDBACK       = 1u << 6,
    APP_TORQUE_INHIBIT_MOTOR_FAULT    = 1u << 7,
    APP_TORQUE_INHIBIT_CURRENT        = 1u << 8,
    APP_TORQUE_INHIBIT_STICK          = 1u << 9,
    APP_TORQUE_INHIBIT_REARM          = 1u << 10
} app_torque_control_inhibit_t;

typedef struct
{
    uint64_t timestamp_us;
    bool enabled;
    bool stick_must_remain_neutral;
} app_torque_control_request_t;

typedef struct
{
    uint64_t timestamp_us;
    float requested_current_a[APP_ACTUATOR_COUNT];
    float approved_current_a[APP_ACTUATOR_COUNT];
    uint32_t inhibit_flags;
    app_torque_control_state_t state;
    bool transmit_enabled;
    bool motion_allowed;
} app_torque_control_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
