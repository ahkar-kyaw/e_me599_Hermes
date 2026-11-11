#include "protocols/proto_cubemars_ak.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PROTO_CUBEMARS_AK_SERVO_MODE_CURRENT 1u

#define PROTO_CUBEMARS_AK_CURRENT_SCALE_A         1000.0f
#define PROTO_CUBEMARS_AK_POSITION_FEEDBACK_DEG   0.1f
#define PROTO_CUBEMARS_AK_VELOCITY_FEEDBACK_ERPM  10.0f
#define PROTO_CUBEMARS_AK_CURRENT_FEEDBACK_A       0.01f

#define PROTO_CUBEMARS_AK_CURRENT_MIN_A -60.0f
#define PROTO_CUBEMARS_AK_CURRENT_MAX_A  60.0f

static proto_cubemars_ak_status_t
proto_cubemars_ak_encode_i32(uint8_t function_id,
                             uint8_t motor_id,
                             float value,
                             double scale,
                             if_can_frame_t *frame);

static bool proto_cubemars_ak_float_fits_i32(double value);
static void proto_cubemars_ak_write_i32_be(uint8_t *data, int32_t value);
static int16_t proto_cubemars_ak_read_i16_be(const uint8_t *data);

uint32_t proto_cubemars_ak_make_servo_id(uint8_t function_id,
                                         uint8_t motor_id)
{
    return ((uint32_t)function_id << 8u) | (uint32_t)motor_id;
}

uint8_t proto_cubemars_ak_get_function_id(const if_can_frame_t *frame)
{
    if (frame == NULL)
    {
        return 0u;
    }

    return (uint8_t)((frame->id >> 8u) & 0xFFu);
}

uint8_t proto_cubemars_ak_get_motor_id(const if_can_frame_t *frame)
{
    if (frame == NULL)
    {
        return 0u;
    }

    return (uint8_t)(frame->id & 0xFFu);
}

proto_cubemars_ak_status_t
proto_cubemars_ak_encode_servo_current(uint8_t motor_id,
                                       float current_a,
                                       if_can_frame_t *frame)
{
    if (frame == NULL)
    {
        return PROTO_CUBEMARS_AK_ERROR_ARGUMENT;
    }

    if ((current_a < PROTO_CUBEMARS_AK_CURRENT_MIN_A) ||
        (current_a > PROTO_CUBEMARS_AK_CURRENT_MAX_A))
    {
        return PROTO_CUBEMARS_AK_ERROR_RANGE;
    }

    return proto_cubemars_ak_encode_i32(
        PROTO_CUBEMARS_AK_SERVO_MODE_CURRENT,
        motor_id,
        current_a,
        PROTO_CUBEMARS_AK_CURRENT_SCALE_A,
        frame);
}

proto_cubemars_ak_status_t
proto_cubemars_ak_decode_servo_feedback(
    const if_can_frame_t *frame,
    uint8_t expected_motor_id,
    proto_cubemars_ak_feedback_t *feedback)
{
    if ((frame == NULL) || (feedback == NULL))
    {
        return PROTO_CUBEMARS_AK_ERROR_ARGUMENT;
    }

    if (!frame->is_extended ||
        frame->is_remote ||
        (frame->data_length != 8u) ||
        (proto_cubemars_ak_get_function_id(frame) !=
         PROTO_CUBEMARS_AK_SERVO_FEEDBACK_FUNCTION_ID) ||
        (proto_cubemars_ak_get_motor_id(frame) != expected_motor_id))
    {
        return PROTO_CUBEMARS_AK_ERROR_FRAME;
    }

    feedback->position_deg =
        (float)proto_cubemars_ak_read_i16_be(&frame->data[0]) *
        PROTO_CUBEMARS_AK_POSITION_FEEDBACK_DEG;
    feedback->velocity_erpm =
        (float)proto_cubemars_ak_read_i16_be(&frame->data[2]) *
        PROTO_CUBEMARS_AK_VELOCITY_FEEDBACK_ERPM;
    feedback->current_a =
        (float)proto_cubemars_ak_read_i16_be(&frame->data[4]) *
        PROTO_CUBEMARS_AK_CURRENT_FEEDBACK_A;
    feedback->temperature_c = (int8_t)frame->data[6];
    feedback->fault_code = frame->data[7];

    return PROTO_CUBEMARS_AK_OK;
}

static proto_cubemars_ak_status_t
proto_cubemars_ak_encode_i32(uint8_t function_id,
                             uint8_t motor_id,
                             float value,
                             double scale,
                             if_can_frame_t *frame)
{
    if (frame == NULL)
    {
        return PROTO_CUBEMARS_AK_ERROR_ARGUMENT;
    }

    const double scaled = (double)value * scale;

    if (!proto_cubemars_ak_float_fits_i32(scaled))
    {
        return PROTO_CUBEMARS_AK_ERROR_RANGE;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = proto_cubemars_ak_make_servo_id(function_id, motor_id);
    frame->data_length = 4u;
    frame->is_extended = true;

    proto_cubemars_ak_write_i32_be(
        frame->data,
        (int32_t)llround(scaled));

    return PROTO_CUBEMARS_AK_OK;
}

static bool proto_cubemars_ak_float_fits_i32(double value)
{
    return isfinite(value) &&
           (value >= (double)INT32_MIN) &&
           (value <= (double)INT32_MAX);
}

static void proto_cubemars_ak_write_i32_be(uint8_t *data, int32_t value)
{
    const uint32_t encoded = (uint32_t)value;

    data[0] = (uint8_t)(encoded >> 24u);
    data[1] = (uint8_t)(encoded >> 16u);
    data[2] = (uint8_t)(encoded >> 8u);
    data[3] = (uint8_t)encoded;
}

static int16_t proto_cubemars_ak_read_i16_be(const uint8_t *data)
{
    const uint16_t encoded =
        ((uint16_t)data[0] << 8u) |
        (uint16_t)data[1];

    return (int16_t)encoded;
}
