#include "estimation/est_collinear_mecanum.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOLERANCE 1.0e-4f

static void expect_near(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        (void)fprintf(stderr,
                      "%s: expected %.6f, got %.6f\n",
                      name,
                      (double)expected,
                      (double)actual);
        exit(EXIT_FAILURE);
    }
}

static est_collinear_mecanum_t make_estimator(void)
{
    const est_collinear_mecanum_config_t config =
    {
        .wheel_radius_m = 0.052f,
        .wheel_offset_m = {0.095f, 0.033f, -0.033f, -0.095f},
        .roller_angle_rad =
        {
             0.7853982f,
            -0.7853982f,
             0.7853982f,
            -0.7853982f,
        },
        .max_dt_s = 0.020f,
    };
    est_collinear_mecanum_t estimator;

    if (!est_collinear_mecanum_init(&estimator, &config))
    {
        (void)fprintf(stderr, "estimator initialization failed\n");
        exit(EXIT_FAILURE);
    }

    return estimator;
}

static void test_forward_velocity(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    const float wheel_velocity[4] =
    {
        1.0f / 0.052f,
        1.0f / 0.052f,
        1.0f / 0.052f,
        1.0f / 0.052f,
    };

    if (!est_collinear_mecanum_update(&estimator,
                                       wheel_velocity,
                                       0.005f,
                                       &output))
    {
        exit(EXIT_FAILURE);
    }

    expect_near("forward body velocity", output.body_forward_mps, 1.0f);
    expect_near("forward lateral velocity", output.body_left_mps, 0.0f);
    expect_near("forward yaw rate", output.yaw_rate_rps, 0.0f);
    expect_near("forward position", output.position_x_m, 0.005f);
}

static void test_lateral_velocity(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    const float wheel_velocity[4] =
    {
         1.0f / 0.052f,
        -1.0f / 0.052f,
         1.0f / 0.052f,
        -1.0f / 0.052f,
    };

    if (!est_collinear_mecanum_update(&estimator,
                                       wheel_velocity,
                                       0.005f,
                                       &output))
    {
        exit(EXIT_FAILURE);
    }

    expect_near("lateral forward velocity", output.body_forward_mps, 0.0f);
    expect_near("lateral body velocity", output.body_left_mps, 1.0f);
    expect_near("lateral yaw rate", output.yaw_rate_rps, 0.0f);
    expect_near("lateral position", output.position_y_m, 0.005f);
}

static void test_yaw_rate(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    const float wheel_velocity[4] =
    {
         0.095f / 0.052f,
         0.033f / 0.052f,
        -0.033f / 0.052f,
        -0.095f / 0.052f,
    };

    if (!est_collinear_mecanum_update(&estimator,
                                       wheel_velocity,
                                       0.005f,
                                       &output))
    {
        exit(EXIT_FAILURE);
    }

    expect_near("yaw forward velocity", output.body_forward_mps, 0.0f);
    expect_near("yaw lateral velocity", output.body_left_mps, 0.0f);
    expect_near("yaw rate", output.yaw_rate_rps, 1.0f);
    expect_near("yaw angle", output.yaw_rad, 0.005f);
}

int main(void)
{
    test_forward_velocity();
    test_lateral_velocity();
    test_yaw_rate();
    return EXIT_SUCCESS;
}
