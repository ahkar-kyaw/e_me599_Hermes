#ifndef TEST_ACTUATOR_CURRENT_H
#define TEST_ACTUATOR_CURRENT_H

#include "app/app_actuator_test_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float current_a;
    uint32_t prepare_ms;
    uint32_t positive_ms;
    uint32_t neutral_ms;
    uint32_t negative_ms;
} test_actuator_current_config_t;

typedef struct
{
    test_actuator_current_config_t config;
    uint64_t stage_started_us;
    uint32_t handled_start_request_count;
    uint32_t run_count;
    float stage_start_velocity_erpm[APP_ACTUATOR_COUNT];
    float positive_velocity_delta_erpm[APP_ACTUATOR_COUNT];
    float negative_velocity_delta_erpm[APP_ACTUATOR_COUNT];
    app_actuator_test_state_t state;
    bool initialized;
} test_actuator_current_t;

bool test_actuator_current_init(
    test_actuator_current_t *test,
    const test_actuator_current_config_t *config);

void test_actuator_current_update(
    test_actuator_current_t *test,
    bool enabled,
    bool motion_allowed,
    uint32_t start_request_count,
    const actuator_snapshot_t *actuator_snapshot,
    uint64_t now_us,
    app_actuator_test_snapshot_t *snapshot);

#endif
