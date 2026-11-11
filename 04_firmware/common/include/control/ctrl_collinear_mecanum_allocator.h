#ifndef CTRL_COLLINEAR_MECANUM_ALLOCATOR_H
#define CTRL_COLLINEAR_MECANUM_ALLOCATOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTRL_COLLINEAR_MECANUM_WHEEL_COUNT 4u
#define CTRL_COLLINEAR_MECANUM_AXIS_COUNT  3u

typedef enum
{
    CTRL_COLLINEAR_MECANUM_FORWARD = 0,
    CTRL_COLLINEAR_MECANUM_LEFT,
    CTRL_COLLINEAR_MECANUM_YAW
} ctrl_collinear_mecanum_axis_t;

typedef struct
{
    float wheel_offset_m[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT];
    float roller_angle_rad[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT];
    float yaw_scale_m;
    float max_motor_current_a;
} ctrl_collinear_mecanum_allocator_config_t;

typedef struct
{
    ctrl_collinear_mecanum_allocator_config_t config;
    float axis_to_wheel[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT]
                       [CTRL_COLLINEAR_MECANUM_AXIS_COUNT];
    bool initialized;
} ctrl_collinear_mecanum_allocator_t;

bool ctrl_collinear_mecanum_allocator_init(
    ctrl_collinear_mecanum_allocator_t *allocator,
    const ctrl_collinear_mecanum_allocator_config_t *config);

bool ctrl_collinear_mecanum_allocator_update(
    const ctrl_collinear_mecanum_allocator_t *allocator,
    const float axis_current_a[CTRL_COLLINEAR_MECANUM_AXIS_COUNT],
    float wheel_current_a[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT],
    bool *saturated);

#ifdef __cplusplus
}
#endif

#endif
