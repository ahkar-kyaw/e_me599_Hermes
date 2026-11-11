#include "control/ctrl_collinear_mecanum_allocator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CTRL_ALLOCATOR_MIN_DET  1.0e-8f
#define CTRL_ALLOCATOR_MIN_SINE 1.0e-4f

static bool ctrl_allocator_config_valid(
    const ctrl_collinear_mecanum_allocator_config_t *config);
static bool ctrl_allocator_invert_3x3(
    const float matrix[3][3],
    float inverse[3][3]);

bool ctrl_collinear_mecanum_allocator_init(
    ctrl_collinear_mecanum_allocator_t *allocator,
    const ctrl_collinear_mecanum_allocator_config_t *config)
{
    if ((allocator == NULL) || !ctrl_allocator_config_valid(config))
    {
        return false;
    }

    float wheel_map[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT]
                   [CTRL_COLLINEAR_MECANUM_AXIS_COUNT];
    float normal[3][3] = {{0.0f}};
    float normal_inverse[3][3];

    for (uint32_t wheel = 0u;
         wheel < CTRL_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        const float angle = config->roller_angle_rad[wheel];

        wheel_map[wheel][CTRL_COLLINEAR_MECANUM_FORWARD] = 1.0f;
        wheel_map[wheel][CTRL_COLLINEAR_MECANUM_LEFT] =
            cosf(angle) / sinf(angle);
        wheel_map[wheel][CTRL_COLLINEAR_MECANUM_YAW] =
            config->wheel_offset_m[wheel] / config->yaw_scale_m;

        for (uint32_t row = 0u; row < 3u; row++)
        {
            for (uint32_t column = 0u; column < 3u; column++)
            {
                normal[row][column] +=
                    wheel_map[wheel][row] * wheel_map[wheel][column];
            }
        }
    }

    if (!ctrl_allocator_invert_3x3(normal, normal_inverse))
    {
        return false;
    }

    memset(allocator, 0, sizeof(*allocator));
    allocator->config = *config;

    for (uint32_t wheel = 0u;
         wheel < CTRL_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        for (uint32_t axis = 0u;
             axis < CTRL_COLLINEAR_MECANUM_AXIS_COUNT;
             axis++)
        {
            for (uint32_t map_axis = 0u;
                 map_axis < CTRL_COLLINEAR_MECANUM_AXIS_COUNT;
                 map_axis++)
            {
                allocator->axis_to_wheel[wheel][axis] +=
                    wheel_map[wheel][map_axis] *
                    normal_inverse[map_axis][axis] *
                    normal[axis][axis];
            }
        }
    }

    allocator->initialized = true;
    return true;
}

bool ctrl_collinear_mecanum_allocator_update(
    const ctrl_collinear_mecanum_allocator_t *allocator,
    const float axis_current_a[CTRL_COLLINEAR_MECANUM_AXIS_COUNT],
    float wheel_current_a[CTRL_COLLINEAR_MECANUM_WHEEL_COUNT],
    bool *saturated)
{
    if (wheel_current_a != NULL)
    {
        memset(wheel_current_a,
               0,
               sizeof(float) * CTRL_COLLINEAR_MECANUM_WHEEL_COUNT);
    }

    if (saturated != NULL)
    {
        *saturated = false;
    }

    if ((allocator == NULL) || !allocator->initialized ||
        (axis_current_a == NULL) || (wheel_current_a == NULL) ||
        (saturated == NULL))
    {
        return false;
    }

    float peak_current_a = 0.0f;

    for (uint32_t axis = 0u;
         axis < CTRL_COLLINEAR_MECANUM_AXIS_COUNT;
         axis++)
    {
        if (!isfinite(axis_current_a[axis]))
        {
            return false;
        }
    }

    for (uint32_t wheel = 0u;
         wheel < CTRL_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        for (uint32_t axis = 0u;
             axis < CTRL_COLLINEAR_MECANUM_AXIS_COUNT;
             axis++)
        {
            wheel_current_a[wheel] +=
                allocator->axis_to_wheel[wheel][axis] *
                axis_current_a[axis];
        }

        peak_current_a = fmaxf(peak_current_a,
                               fabsf(wheel_current_a[wheel]));
    }

    if (peak_current_a > allocator->config.max_motor_current_a)
    {
        const float scale = allocator->config.max_motor_current_a /
            peak_current_a;

        for (uint32_t wheel = 0u;
             wheel < CTRL_COLLINEAR_MECANUM_WHEEL_COUNT;
             wheel++)
        {
            wheel_current_a[wheel] *= scale;
        }

        *saturated = true;
    }

    return true;
}

static bool ctrl_allocator_config_valid(
    const ctrl_collinear_mecanum_allocator_config_t *config)
{
    if ((config == NULL) || !isfinite(config->yaw_scale_m) ||
        !isfinite(config->max_motor_current_a) ||
        (config->yaw_scale_m <= 0.0f) ||
        (config->max_motor_current_a <= 0.0f))
    {
        return false;
    }

    for (uint32_t wheel = 0u;
         wheel < CTRL_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        if (!isfinite(config->wheel_offset_m[wheel]) ||
            !isfinite(config->roller_angle_rad[wheel]) ||
            (fabsf(sinf(config->roller_angle_rad[wheel])) <
             CTRL_ALLOCATOR_MIN_SINE))
        {
            return false;
        }
    }

    return true;
}

static bool ctrl_allocator_invert_3x3(
    const float matrix[3][3],
    float inverse[3][3])
{
    const float determinant =
        (matrix[0][0] *
         ((matrix[1][1] * matrix[2][2]) -
          (matrix[1][2] * matrix[2][1]))) -
        (matrix[0][1] *
         ((matrix[1][0] * matrix[2][2]) -
          (matrix[1][2] * matrix[2][0]))) +
        (matrix[0][2] *
         ((matrix[1][0] * matrix[2][1]) -
          (matrix[1][1] * matrix[2][0])));

    if (!isfinite(determinant) ||
        (fabsf(determinant) < CTRL_ALLOCATOR_MIN_DET))
    {
        return false;
    }

    const float inverse_determinant = 1.0f / determinant;

    inverse[0][0] = ((matrix[1][1] * matrix[2][2]) -
                     (matrix[1][2] * matrix[2][1])) *
                    inverse_determinant;
    inverse[0][1] = ((matrix[0][2] * matrix[2][1]) -
                     (matrix[0][1] * matrix[2][2])) *
                    inverse_determinant;
    inverse[0][2] = ((matrix[0][1] * matrix[1][2]) -
                     (matrix[0][2] * matrix[1][1])) *
                    inverse_determinant;
    inverse[1][0] = ((matrix[1][2] * matrix[2][0]) -
                     (matrix[1][0] * matrix[2][2])) *
                    inverse_determinant;
    inverse[1][1] = ((matrix[0][0] * matrix[2][2]) -
                     (matrix[0][2] * matrix[2][0])) *
                    inverse_determinant;
    inverse[1][2] = ((matrix[0][2] * matrix[1][0]) -
                     (matrix[0][0] * matrix[1][2])) *
                    inverse_determinant;
    inverse[2][0] = ((matrix[1][0] * matrix[2][1]) -
                     (matrix[1][1] * matrix[2][0])) *
                    inverse_determinant;
    inverse[2][1] = ((matrix[0][1] * matrix[2][0]) -
                     (matrix[0][0] * matrix[2][1])) *
                    inverse_determinant;
    inverse[2][2] = ((matrix[0][0] * matrix[1][1]) -
                     (matrix[0][1] * matrix[1][0])) *
                    inverse_determinant;

    return true;
}
