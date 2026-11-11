#include "control/ctrl_collinear_mecanum_allocator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOLERANCE 1.0e-4f

static void expect_near(float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        (void)fprintf(stderr,
                      "expected %.6f, got %.6f\n",
                      (double)expected,
                      (double)actual);
        exit(EXIT_FAILURE);
    }
}

static void expect_generalized_current(
    const ctrl_collinear_mecanum_allocator_config_t *config,
    const float wheel_current[4],
    const float expected[3])
{
    float generalized[3] = {0.0f};

    for (uint32_t wheel = 0u; wheel < 4u; wheel++)
    {
        const float angle = config->roller_angle_rad[wheel];
        const float row[3] =
        {
            1.0f,
            cosf(angle) / sinf(angle),
            config->wheel_offset_m[wheel] / config->yaw_scale_m,
        };

        for (uint32_t axis = 0u; axis < 3u; axis++)
        {
            generalized[axis] += row[axis] * wheel_current[wheel];
        }
    }

    for (uint32_t axis = 0u; axis < 3u; axis++)
    {
        expect_near(generalized[axis], expected[axis]);
    }
}

int main(void)
{
    const ctrl_collinear_mecanum_allocator_config_t config =
    {
        .wheel_offset_m = {0.095f, 0.033f, -0.033f, -0.095f},
        .roller_angle_rad =
        {
             0.7853982f,
            -0.7853982f,
             0.7853982f,
            -0.7853982f,
        },
        .yaw_scale_m = 0.095f,
        .max_motor_current_a = 1.0f,
    };
    ctrl_collinear_mecanum_allocator_t allocator;
    float wheel_current[4];
    bool saturated = false;

    if (!ctrl_collinear_mecanum_allocator_init(&allocator, &config))
    {
        return EXIT_FAILURE;
    }

    const float forward[3] = {0.5f, 0.0f, 0.0f};
    (void)ctrl_collinear_mecanum_allocator_update(
        &allocator, forward, wheel_current, &saturated);

    for (uint32_t wheel = 0u; wheel < 4u; wheel++)
    {
        expect_near(wheel_current[wheel], 0.5f);
    }

    const float lateral[3] = {0.0f, 0.25f, 0.0f};
    (void)ctrl_collinear_mecanum_allocator_update(
        &allocator, lateral, wheel_current, &saturated);
    const float expected_lateral[3] = {0.0f, 1.0f, 0.0f};
    expect_generalized_current(&config,
                               wheel_current,
                               expected_lateral);

    const float yaw[3] = {0.0f, 0.0f, 0.2f};
    (void)ctrl_collinear_mecanum_allocator_update(
        &allocator, yaw, wheel_current, &saturated);
    float yaw_normal = 0.0f;

    for (uint32_t wheel = 0u; wheel < 4u; wheel++)
    {
        const float normalized_offset = config.wheel_offset_m[wheel] /
            config.yaw_scale_m;
        yaw_normal += normalized_offset * normalized_offset;
    }

    const float expected_yaw[3] =
    {
        0.0f,
        0.0f,
        yaw_normal * yaw[2],
    };
    expect_generalized_current(&config, wheel_current, expected_yaw);

    const float combined[3] = {1.0f, 1.0f, 1.0f};
    (void)ctrl_collinear_mecanum_allocator_update(
        &allocator, combined, wheel_current, &saturated);

    if (!saturated)
    {
        return EXIT_FAILURE;
    }

    for (uint32_t wheel = 0u; wheel < 4u; wheel++)
    {
        if (fabsf(wheel_current[wheel]) > 1.0f)
        {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
