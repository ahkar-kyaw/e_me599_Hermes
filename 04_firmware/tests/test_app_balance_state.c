#include "app/app_balance_state.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_TOLERANCE 1.0e-6f

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

int main(void)
{
    imu_snapshot_t imu;
    safety_imu_status_t status;
    float lean_rad = 0.0f;
    float lean_rate_rps = 0.0f;

    memset(&imu, 0, sizeof(imu));
    memset(&status, 0, sizeof(status));
    imu.attitude.valid = true;
    imu.attitude.pitch_rad = 0.2f;
    imu.attitude.pitch_rate_rps = -0.3f;

    if (!app_balance_state_get_lean(&imu,
                                     &status,
                                     CTRL_BALANCE_LEAN_PITCH,
                                     -1.0f,
                                     &lean_rad,
                                     &lean_rate_rps))
    {
        return EXIT_FAILURE;
    }

    expect_near(lean_rad, -0.2f);
    expect_near(lean_rate_rps, 0.3f);

    if (app_balance_state_get_lean(&imu,
                                    &status,
                                    CTRL_BALANCE_LEAN_PITCH,
                                    0.0f,
                                    &lean_rad,
                                    &lean_rate_rps))
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
