#include "estimation/est_collinear_mecanum.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define EST_COLLINEAR_MECANUM_PI      3.1415926536f
#define EST_COLLINEAR_MECANUM_TWO_PI  6.2831853072f
#define EST_COLLINEAR_MECANUM_MIN_DET 1.0e-8f
#define EST_COLLINEAR_MECANUM_MIN_SINE 1.0e-4f

static bool est_collinear_mecanum_config_valid(
    const est_collinear_mecanum_config_t *config);
static bool est_collinear_mecanum_invert_3x3(
    const float matrix[3][3],
    float inverse[3][3]);
static float est_collinear_mecanum_wrap_pi(float angle_rad);

bool est_collinear_mecanum_init(
    est_collinear_mecanum_t *estimator,
    const est_collinear_mecanum_config_t *config)
{
    if ((estimator == NULL) ||
        !est_collinear_mecanum_config_valid(config))
    {
        return false;
    }

    float wheel_map[EST_COLLINEAR_MECANUM_WHEEL_COUNT][3];
    float normal_matrix[3][3] = {{0.0f}};
    float normal_inverse[3][3];

    for (uint32_t wheel = 0u;
         wheel < EST_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        const float sine_angle = sinf(config->roller_angle_rad[wheel]);

        wheel_map[wheel][0] = 1.0f;
        wheel_map[wheel][1] =
            cosf(config->roller_angle_rad[wheel]) / sine_angle;
        wheel_map[wheel][2] = config->wheel_offset_m[wheel];

        for (uint32_t row = 0u; row < 3u; row++)
        {
            for (uint32_t column = 0u; column < 3u; column++)
            {
                normal_matrix[row][column] +=
                    wheel_map[wheel][row] * wheel_map[wheel][column];
            }
        }
    }

    if (!est_collinear_mecanum_invert_3x3(normal_matrix,
                                           normal_inverse))
    {
        return false;
    }

    memset(estimator, 0, sizeof(*estimator));
    estimator->config = *config;

    for (uint32_t body_axis = 0u; body_axis < 3u; body_axis++)
    {
        for (uint32_t wheel = 0u;
             wheel < EST_COLLINEAR_MECANUM_WHEEL_COUNT;
             wheel++)
        {
            for (uint32_t map_axis = 0u; map_axis < 3u; map_axis++)
            {
                estimator->wheel_to_body[body_axis][wheel] +=
                    config->wheel_radius_m *
                    normal_inverse[body_axis][map_axis] *
                    wheel_map[wheel][map_axis];
            }
        }
    }

    estimator->initialized = true;
    return true;
}

void est_collinear_mecanum_reset(est_collinear_mecanum_t *estimator)
{
    if ((estimator == NULL) || !estimator->initialized)
    {
        return;
    }

    estimator->position_x_m = 0.0f;
    estimator->position_y_m = 0.0f;
    estimator->yaw_rad = 0.0f;
}

bool est_collinear_mecanum_update(
    est_collinear_mecanum_t *estimator,
    const float wheel_velocity_radps[EST_COLLINEAR_MECANUM_WHEEL_COUNT],
    float dt_s,
    est_collinear_mecanum_output_t *output)
{
    if (output != NULL)
    {
        memset(output, 0, sizeof(*output));
    }

    if ((estimator == NULL) || !estimator->initialized ||
        (wheel_velocity_radps == NULL) || (output == NULL) ||
        !isfinite(dt_s) || (dt_s <= 0.0f) ||
        (dt_s > estimator->config.max_dt_s))
    {
        return false;
    }

    float body_velocity[3] = {0.0f};

    for (uint32_t wheel = 0u;
         wheel < EST_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        if (!isfinite(wheel_velocity_radps[wheel]))
        {
            return false;
        }

        for (uint32_t body_axis = 0u; body_axis < 3u; body_axis++)
        {
            body_velocity[body_axis] +=
                estimator->wheel_to_body[body_axis][wheel] *
                wheel_velocity_radps[wheel];
        }
    }

    const float yaw_midpoint = estimator->yaw_rad +
        (0.5f * body_velocity[2] * dt_s);
    const float cosine_yaw = cosf(yaw_midpoint);
    const float sine_yaw = sinf(yaw_midpoint);
    const float velocity_x_mps =
        (cosine_yaw * body_velocity[0]) -
        (sine_yaw * body_velocity[1]);
    const float velocity_y_mps =
        (sine_yaw * body_velocity[0]) +
        (cosine_yaw * body_velocity[1]);

    estimator->position_x_m += velocity_x_mps * dt_s;
    estimator->position_y_m += velocity_y_mps * dt_s;
    estimator->yaw_rad = est_collinear_mecanum_wrap_pi(
        estimator->yaw_rad + (body_velocity[2] * dt_s));

    output->valid = true;
    output->position_x_m = estimator->position_x_m;
    output->position_y_m = estimator->position_y_m;
    output->yaw_rad = estimator->yaw_rad;
    output->velocity_x_mps = velocity_x_mps;
    output->velocity_y_mps = velocity_y_mps;
    output->yaw_rate_rps = body_velocity[2];
    output->body_forward_mps = body_velocity[0];
    output->body_left_mps = body_velocity[1];

    return true;
}

static bool est_collinear_mecanum_config_valid(
    const est_collinear_mecanum_config_t *config)
{
    if ((config == NULL) || !isfinite(config->wheel_radius_m) ||
        !isfinite(config->max_dt_s) ||
        (config->wheel_radius_m <= 0.0f) ||
        (config->max_dt_s <= 0.0f))
    {
        return false;
    }

    for (uint32_t wheel = 0u;
         wheel < EST_COLLINEAR_MECANUM_WHEEL_COUNT;
         wheel++)
    {
        if (!isfinite(config->wheel_offset_m[wheel]) ||
            !isfinite(config->roller_angle_rad[wheel]) ||
            (fabsf(sinf(config->roller_angle_rad[wheel])) <
             EST_COLLINEAR_MECANUM_MIN_SINE))
        {
            return false;
        }
    }

    return true;
}

static bool est_collinear_mecanum_invert_3x3(
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
        (fabsf(determinant) < EST_COLLINEAR_MECANUM_MIN_DET))
    {
        return false;
    }

    const float inverse_determinant = 1.0f / determinant;

    inverse[0][0] =
        ((matrix[1][1] * matrix[2][2]) -
         (matrix[1][2] * matrix[2][1])) * inverse_determinant;
    inverse[0][1] =
        ((matrix[0][2] * matrix[2][1]) -
         (matrix[0][1] * matrix[2][2])) * inverse_determinant;
    inverse[0][2] =
        ((matrix[0][1] * matrix[1][2]) -
         (matrix[0][2] * matrix[1][1])) * inverse_determinant;
    inverse[1][0] =
        ((matrix[1][2] * matrix[2][0]) -
         (matrix[1][0] * matrix[2][2])) * inverse_determinant;
    inverse[1][1] =
        ((matrix[0][0] * matrix[2][2]) -
         (matrix[0][2] * matrix[2][0])) * inverse_determinant;
    inverse[1][2] =
        ((matrix[0][2] * matrix[1][0]) -
         (matrix[0][0] * matrix[1][2])) * inverse_determinant;
    inverse[2][0] =
        ((matrix[1][0] * matrix[2][1]) -
         (matrix[1][1] * matrix[2][0])) * inverse_determinant;
    inverse[2][1] =
        ((matrix[0][1] * matrix[2][0]) -
         (matrix[0][0] * matrix[2][1])) * inverse_determinant;
    inverse[2][2] =
        ((matrix[0][0] * matrix[1][1]) -
         (matrix[0][1] * matrix[1][0])) * inverse_determinant;

    return true;
}

static float est_collinear_mecanum_wrap_pi(float angle_rad)
{
    while (angle_rad > EST_COLLINEAR_MECANUM_PI)
    {
        angle_rad -= EST_COLLINEAR_MECANUM_TWO_PI;
    }

    while (angle_rad < -EST_COLLINEAR_MECANUM_PI)
    {
        angle_rad += EST_COLLINEAR_MECANUM_TWO_PI;
    }

    return angle_rad;
}
