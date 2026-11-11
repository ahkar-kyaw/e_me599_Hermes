#ifndef CONFIG_ACTUATOR_H
#define CONFIG_ACTUATOR_H

#include "app/app_actuator_types.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_ACTUATOR_CAN_BITRATE          1000000u
#define APP_ACTUATOR_FEEDBACK_MAX_AGE_US   100000u
#define APP_ACTUATOR_CAN_RETRY_MS             1000u
#define APP_ACTUATOR_CAN_TX_TIMEOUT_MS             5u
#define APP_ACTUATOR_TASK_PERIOD_MS                 1u
#define APP_ACTUATOR_RX_FRAMES_PER_CYCLE           16u
#define APP_ACTUATOR_STATUS_PERIOD_MS              10u
#define APP_ACTUATOR_COMMAND_PERIOD_MS              5u
#define APP_ACTUATOR_COMMAND_MAX_AGE_US          20000u
#define APP_ACTUATOR_COMMAND_STOP_HOLD_US       200000u

#define APP_ACTUATOR_1_ENABLED true
#define APP_ACTUATOR_1_CAN_ID  1u
#define APP_ACTUATOR_1_DIRECTION_SIGN -1

#define APP_ACTUATOR_2_ENABLED true
#define APP_ACTUATOR_2_CAN_ID  20u
#define APP_ACTUATOR_2_DIRECTION_SIGN -1

#define APP_ACTUATOR_3_ENABLED true
#define APP_ACTUATOR_3_CAN_ID  3u
#define APP_ACTUATOR_3_DIRECTION_SIGN  1

#define APP_ACTUATOR_4_ENABLED true
#define APP_ACTUATOR_4_CAN_ID  4u
#define APP_ACTUATOR_4_DIRECTION_SIGN  1

typedef struct
{
    uint8_t motor_id;
    int8_t direction_sign;
    bool enabled;
} config_actuator_entry_t;

static const config_actuator_entry_t
    APP_ACTUATOR_CONFIG[APP_ACTUATOR_COUNT] =
{
    {
        APP_ACTUATOR_1_CAN_ID,
        APP_ACTUATOR_1_DIRECTION_SIGN,
        APP_ACTUATOR_1_ENABLED,
    },
    {
        APP_ACTUATOR_2_CAN_ID,
        APP_ACTUATOR_2_DIRECTION_SIGN,
        APP_ACTUATOR_2_ENABLED,
    },
    {
        APP_ACTUATOR_3_CAN_ID,
        APP_ACTUATOR_3_DIRECTION_SIGN,
        APP_ACTUATOR_3_ENABLED,
    },
    {
        APP_ACTUATOR_4_CAN_ID,
        APP_ACTUATOR_4_DIRECTION_SIGN,
        APP_ACTUATOR_4_ENABLED,
    },
};

#endif
