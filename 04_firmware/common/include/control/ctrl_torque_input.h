#ifndef CTRL_TORQUE_INPUT_H
#define CTRL_TORQUE_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t channel_min;
    uint16_t channel_max;
    uint16_t neutral_low;
    uint16_t neutral_high;
    float max_current_a;
    float slew_rate_a_per_s;
    bool positive_high;
} ctrl_torque_input_config_t;

typedef struct
{
    ctrl_torque_input_config_t config;
    float current_a;
    bool initialized;
} ctrl_torque_input_t;

bool ctrl_torque_input_init(
    ctrl_torque_input_t *input,
    const ctrl_torque_input_config_t *config);

void ctrl_torque_input_reset(ctrl_torque_input_t *input);

bool ctrl_torque_input_update(ctrl_torque_input_t *input,
                              uint16_t channel_value,
                              float dt_s,
                              float *current_a);

#ifdef __cplusplus
}
#endif

#endif
