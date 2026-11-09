#ifndef EST_COLLINEAR_MECANUM_H
#define EST_COLLINEAR_MECANUM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EST_COLLINEAR_MECANUM_WHEEL_COUNT 4u

typedef struct
{
    float wheel_radius_m;
    float wheel_lateral_m[EST_COLLINEAR_MECANUM_WHEEL_COUNT];
    float roller_lateral_factor[EST_COLLINEAR_MECANUM_WHEEL_COUNT];
    float max_dt_s;
} est_collinear_mecanum_config_t;

typedef struct
{
    bool valid;

    float position_x_m;
    float position_y_m;
    float yaw_rad;

    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_rps;

    float body_forward_mps;
    float body_left_mps;
} est_collinear_mecanum_output_t;

typedef struct
{
    est_collinear_mecanum_config_t config;
    float wheel_to_body[3][EST_COLLINEAR_MECANUM_WHEEL_COUNT];

    float position_x_m;
    float position_y_m;
    float yaw_rad;

    bool initialized;
} est_collinear_mecanum_t;

bool est_collinear_mecanum_init(
    est_collinear_mecanum_t *estimator,
    const est_collinear_mecanum_config_t *config);

void est_collinear_mecanum_reset(est_collinear_mecanum_t *estimator);

bool est_collinear_mecanum_update(
    est_collinear_mecanum_t *estimator,
    const float wheel_velocity_radps[EST_COLLINEAR_MECANUM_WHEEL_COUNT],
    float dt_s,
    est_collinear_mecanum_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
