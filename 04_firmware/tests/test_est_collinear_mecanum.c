#include "estimation/est_collinear_mecanum.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_TOLERANCE 1.0e-4f
#define TEST_WHEEL_RADIUS_M 0.052f

static const float TEST_WHEEL_OFFSET_M[4] =
{
    -0.095f,
    -0.033f,
     0.033f,
     0.095f,
};

static const float TEST_ROLLER_ANGLE_RAD[4] =
{
     0.7853982f,
    -0.7853982f,
     0.7853982f,
    -0.7853982f,
};

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
        .wheel_radius_m = TEST_WHEEL_RADIUS_M,
        .wheel_offset_m =
        {
            TEST_WHEEL_OFFSET_M[0],
            TEST_WHEEL_OFFSET_M[1],
            TEST_WHEEL_OFFSET_M[2],
            TEST_WHEEL_OFFSET_M[3],
        },
        .roller_angle_rad =
        {
            TEST_ROLLER_ANGLE_RAD[0],
            TEST_ROLLER_ANGLE_RAD[1],
            TEST_ROLLER_ANGLE_RAD[2],
            TEST_ROLLER_ANGLE_RAD[3],
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

static void make_wheel_velocity(float body_right_mps,
                                float body_forward_mps,
                                float yaw_rate_rps,
                                float wheel_velocity_radps[4])
{
    for (uint32_t wheel = 0u; wheel < 4u; wheel++)
    {
        const float alpha = TEST_ROLLER_ANGLE_RAD[wheel];

        /* Eq. (8) */
        wheel_velocity_radps[wheel] =
            (-(cosf(alpha) / sinf(alpha)) * body_right_mps +
             body_forward_mps +
             (TEST_WHEEL_OFFSET_M[wheel] * yaw_rate_rps)) /
            TEST_WHEEL_RADIUS_M;
    }
}

static void test_forward_velocity(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    float wheel_velocity[4];

    make_wheel_velocity(0.0f, 1.0f, 0.0f, wheel_velocity);

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
    expect_near("forward world x", output.position_x_m, 0.0f);
    expect_near("forward world y", output.position_y_m, 0.005f);
}

static void test_lateral_velocity(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    float wheel_velocity[4];

    make_wheel_velocity(-1.0f, 0.0f, 0.0f, wheel_velocity);

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
    expect_near("left world x", output.position_x_m, -0.005f);
    expect_near("left world y", output.position_y_m, 0.0f);
}

static void test_yaw_rate(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    float wheel_velocity[4];

    make_wheel_velocity(0.0f, 0.0f, 1.0f, wheel_velocity);

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

static void test_forward_velocity_at_quarter_turn(void)
{
    est_collinear_mecanum_t estimator = make_estimator();
    est_collinear_mecanum_output_t output;
    float wheel_velocity[4];

    estimator.yaw_rad = 1.5707963268f;
    make_wheel_velocity(0.0f, 1.0f, 0.0f, wheel_velocity);

    if (!est_collinear_mecanum_update(&estimator,
                                       wheel_velocity,
                                       0.005f,
                                       &output))
    {
        exit(EXIT_FAILURE);
    }

    expect_near("turned forward world x", output.position_x_m, -0.005f);
    expect_near("turned forward world y", output.position_y_m, 0.0f);
}

int main(void)
{
    test_forward_velocity();
    test_lateral_velocity();
    test_yaw_rate();
    test_forward_velocity_at_quarter_turn();
    return EXIT_SUCCESS;
}
