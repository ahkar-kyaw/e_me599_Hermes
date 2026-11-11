#ifndef TEST_ACTUATOR_DRIVE_H
#define TEST_ACTUATOR_DRIVE_H

#include "app/app_actuator_types.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float max_motor_current_a;
    float current_slew_rate_a_per_s;
} test_actuator_drive_config_t;

typedef struct
{
    test_actuator_drive_config_t config;
    float forward_current_a;
    float turn_current_a;
    bool initialized;
} test_actuator_drive_t;

typedef struct
{
    float motor_current_a[APP_ACTUATOR_COUNT];
    float forward_current_a;
    float turn_current_a;
    bool saturated;
    bool valid;
} test_actuator_drive_output_t;

bool test_actuator_drive_init(
    test_actuator_drive_t *test,
    const test_actuator_drive_config_t *config);

void test_actuator_drive_reset(test_actuator_drive_t *test);

bool test_actuator_drive_update(
    test_actuator_drive_t *test,
    float forward_input,
    float turn_input,
    float dt_s,
    test_actuator_drive_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
