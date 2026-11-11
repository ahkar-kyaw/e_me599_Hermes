#ifndef PROTO_CUBEMARS_AK_H
#define PROTO_CUBEMARS_AK_H

#include "interfaces/if_can.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_CUBEMARS_AK_SERVO_FEEDBACK_FUNCTION_ID 0x29u

typedef enum
{
    PROTO_CUBEMARS_AK_OK = 0,
    PROTO_CUBEMARS_AK_ERROR_ARGUMENT = -1,
    PROTO_CUBEMARS_AK_ERROR_RANGE = -2,
    PROTO_CUBEMARS_AK_ERROR_FRAME = -3
} proto_cubemars_ak_status_t;

typedef enum
{
    PROTO_CUBEMARS_AK_FAULT_NONE = 0,
    PROTO_CUBEMARS_AK_FAULT_MOTOR_OVERTEMPERATURE = 1,
    PROTO_CUBEMARS_AK_FAULT_OVERCURRENT = 2,
    PROTO_CUBEMARS_AK_FAULT_OVERVOLTAGE = 3,
    PROTO_CUBEMARS_AK_FAULT_UNDERVOLTAGE = 4,
    PROTO_CUBEMARS_AK_FAULT_ENCODER = 5,
    PROTO_CUBEMARS_AK_FAULT_MOSFET_OVERTEMPERATURE = 6,
    PROTO_CUBEMARS_AK_FAULT_LOCKED_ROTOR = 7
} proto_cubemars_ak_fault_t;

typedef struct
{
    float position_deg;
    float velocity_erpm;
    float current_a;
    int8_t temperature_c;
    uint8_t fault_code;
} proto_cubemars_ak_feedback_t;

uint32_t proto_cubemars_ak_make_servo_id(uint8_t function_id,
                                         uint8_t motor_id);

uint8_t proto_cubemars_ak_get_function_id(const if_can_frame_t *frame);
uint8_t proto_cubemars_ak_get_motor_id(const if_can_frame_t *frame);

proto_cubemars_ak_status_t
proto_cubemars_ak_encode_servo_current(uint8_t motor_id,
                                       float current_a,
                                       if_can_frame_t *frame);

proto_cubemars_ak_status_t
proto_cubemars_ak_decode_servo_feedback(
    const if_can_frame_t *frame,
    uint8_t expected_motor_id,
    proto_cubemars_ak_feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif
