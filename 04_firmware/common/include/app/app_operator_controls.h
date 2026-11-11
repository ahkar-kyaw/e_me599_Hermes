#ifndef APP_OPERATOR_CONTROLS_H
#define APP_OPERATOR_CONTROLS_H

#include "app/app_rc_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t master_channel;
    uint8_t balance_channel;
    uint8_t zero_lean_channel;
    uint8_t forward_channel;
    uint8_t lateral_channel;
    uint8_t yaw_channel;
    uint16_t channel_min;
    uint16_t channel_max;
    uint16_t input_min;
    uint16_t input_mid;
    uint16_t input_max;
    uint16_t input_deadband;
    uint16_t switch_off_threshold;
    uint16_t switch_on_threshold;
    bool master_active_high;
    bool balance_active_high;
    bool zero_lean_active_high;
    bool forward_inverted;
    bool lateral_inverted;
    bool yaw_inverted;
} app_operator_controls_config_t;

typedef struct
{
    bool link_usable;
    bool master_enabled;
    bool balance_enabled;
    bool zero_lean_requested;
    float forward_input;
    float lateral_input;
    float yaw_input;
} app_operator_controls_state_t;

typedef struct
{
    app_operator_controls_config_t config;
    bool initialized;
    bool master_active;
    bool balance_active;
    bool zero_lean_active;
} app_operator_controls_t;

bool app_operator_controls_init(
    app_operator_controls_t *controls,
    const app_operator_controls_config_t *config);

void app_operator_controls_update(
    app_operator_controls_t *controls,
    const rc_snapshot_t *snapshot,
    bool link_usable,
    app_operator_controls_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
