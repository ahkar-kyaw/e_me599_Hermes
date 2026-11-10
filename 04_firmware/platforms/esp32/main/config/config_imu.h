#ifndef APP_CONFIG_IMU_H
#define APP_CONFIG_IMU_H

#include "drivers/drv_ism330dhcx.h"
#include "estimation/est_attitude.h"
#include "estimation/est_imu_mount.h"

#define APP_IMU_TASK_PERIOD_MS              5u
#define APP_IMU_CALIBRATION_SAMPLES        200u

#define APP_IMU_ACCEL_ODR                  DRV_ISM330DHCX_ODR_208_HZ
#define APP_IMU_GYRO_ODR                   DRV_ISM330DHCX_ODR_208_HZ
#define APP_IMU_ACCEL_FS                   DRV_ISM330DHCX_ACCEL_FS_4G
#define APP_IMU_GYRO_FS                    DRV_ISM330DHCX_GYRO_FS_500_DPS

#define APP_IMU_MOUNT_FORWARD_AXIS         EST_IMU_AXIS_SENSOR_Y_NEG
#define APP_IMU_MOUNT_UP_AXIS              EST_IMU_AXIS_SENSOR_Z_POS

#define APP_IMU_ATTITUDE_ALGORITHM         EST_ATTITUDE_ALGORITHM_COMPLEMENTARY
#define APP_IMU_ATTITUDE_SENSOR_MODE       EST_ATTITUDE_SENSOR_MODE_6DOF
#define APP_IMU_COMPLEMENTARY_ALPHA        0.98f
#define APP_IMU_COMPLEMENTARY_MAG_ALPHA    0.98f

#define APP_IMU_MAX_AGE_US                 50000u
#define APP_IMU_MAX_ABS_ROLL_RAD           1.0471976f
#define APP_IMU_MAX_ABS_PITCH_RAD          1.0471976f
#define APP_IMU_MIN_ACCEL_NORM_MPS2        3.0f
#define APP_IMU_MAX_ACCEL_NORM_MPS2        20.0f

#endif
