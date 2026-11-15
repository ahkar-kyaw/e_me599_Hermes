#ifndef APP_ACTUATOR_TEST_TYPES_H
#define APP_ACTUATOR_TEST_TYPES_H

#include "app/app_actuator_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_ACTUATOR_TEST_DISABLED = 0,
    APP_ACTUATOR_TEST_READY,
    APP_ACTUATOR_TEST_PREPARE,
    APP_ACTUATOR_TEST_POSITIVE,
    APP_ACTUATOR_TEST_NEUTRAL,
    APP_ACTUATOR_TEST_NEGATIVE,
    APP_ACTUATOR_TEST_COMPLETE,
    APP_ACTUATOR_TEST_ABORTED
} app_actuator_test_state_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t run_count;
    uint32_t remaining_ms;
    float command_current_a;
    float positive_velocity_delta_erpm[APP_ACTUATOR_COUNT];
    float negative_velocity_delta_erpm[APP_ACTUATOR_COUNT];
    app_actuator_test_state_t state;
    bool active;
} app_actuator_test_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
