# IMU subsystem

The IMU subsystem currently targets the ISM330DHCX over SPI on the ESP32 prototype. The common logic is portable and can be reused on STM32 by adding the appropriate SPI BSP.

## Current hardware path

```text
ESP32 GPIO18    IMU SCLK
ESP32 GPIO19    IMU MISO
ESP32 GPIO23    IMU MOSI
ESP32 GPIO27    IMU CS
ESP32 GPIO34    IMU INT1, defined but unused
ESP32 GPIO35    IMU INT2, defined but unused
```

The current firmware uses polling. INT1 and INT2 are reserved for future data-ready or FIFO interrupts.

## Software pipeline

```text
bsp_imu_spi_esp32
    Initializes ESP32 SPI and provides if_spi_t.

drv_ism330dhcx
    Probes the IMU.
    Configures ODR and full-scale ranges.
    Reads raw temperature, gyro, and accelerometer registers.
    Converts raw data into SI units.

est_imu_mount
    Maps sensor-frame data into robot body-frame data.

est_imu_calibration
    Collects stationary gyro samples at startup.
    Computes gyro bias.
    Subtracts bias from future gyro readings.

est_attitude
    Estimates roll, pitch, yaw, and angular rates.
    Uses a complementary filter now.
    Provides configuration hooks for future Madgwick or Kalman filters.

safety_imu
    Checks whether the IMU state is safe for balance use.

app_balance_state
    Extracts pitch and pitch rate into balance_state_t.
```

## Body frame convention

```text
Body X positive
    robot front

Body Y positive
    robot left

Body Z positive
    robot up
```

Mounting is configured in:

```text
04_firmware/platforms/esp32/main/config/config_imu.h
```

Important settings:

```text
APP_IMU_MOUNT_FORWARD_AXIS
APP_IMU_MOUNT_UP_AXIS
```

The current mapping is:

```c
#define APP_IMU_MOUNT_FORWARD_AXIS EST_IMU_AXIS_SENSOR_Y_NEG
#define APP_IMU_MOUNT_UP_AXIS      EST_IMU_AXIS_SENSOR_Z_POS
```

This mapping was selected from the observed installation where forward tilt
appeared as positive roll and left tilt appeared as positive pitch. After
flashing, verify the body-frame convention before enabling torque:

```text
tilt forward    pitch becomes positive
tilt left       roll becomes negative
stationary      acceleration Z is approximately +9.81 m/s^2
```

If any result is wrong, change only the two mount-axis macros. Do not repair a
mounting error with controller-gain signs.

## IMU hardware configuration

Configured in:

```text
04_firmware/platforms/esp32/main/config/config_imu.h
```

Current settings:

```text
APP_IMU_ACCEL_ODR
APP_IMU_GYRO_ODR
APP_IMU_ACCEL_FS
APP_IMU_GYRO_FS
```

Recommended early settings:

```text
Accelerometer full-scale
    ±4 g

Gyroscope full-scale
    ±500 dps

Task period
    5 ms

Effective polling rate
    about 200 Hz
```

Use a larger accelerometer range only if shock or impact causes saturation. Use a larger gyro range only if fast falls or disturbances saturate the gyro.

## Attitude estimator configuration

Configured in:

```text
APP_IMU_ATTITUDE_ALGORITHM
APP_IMU_ATTITUDE_SENSOR_MODE
APP_IMU_COMPLEMENTARY_ALPHA
APP_IMU_COMPLEMENTARY_MAG_ALPHA
```

Current mode:

```text
Algorithm
    complementary filter

Sensor mode
    6DOF accel plus gyro
```

Future 9DOF support is represented by the estimator input structure, which includes optional magnetometer data. A magnetometer driver and magnetometer calibration are not implemented yet.

## Snapshot output

Published type:

```text
imu_snapshot_t
```

Main fields:

```text
timestamp_us
valid
calibrated
sample_count
read_error_count
gyro_bias_rps
raw
data
attitude
```

## Safety status

Checked type:

```text
safety_imu_status_t
```

Main checks:

```text
snapshot validity
calibration complete
attitude validity
timestamp freshness
roll and pitch angle limits
acceleration magnitude range
```

## Balance-state output

Control-facing type:

```text
balance_state_t
```

Main fields:

```text
valid
timestamp_us
position_x_m
position_y_m
yaw_rad
lean_rad
velocity_x_mps
velocity_y_mps
yaw_rate_rps
lean_rate_rps
```

Balance control should use `balance_state_t`, not the full IMU snapshot.

## Startup behavior

```text
1. task_imu starts.
2. SPI backend initializes.
3. ISM330DHCX driver initializes.
4. Gyro bias calibration runs while the IMU is stationary.
5. Attitude estimator resets after calibration.
6. Valid snapshots are published.
```

During calibration:

```text
imu_snapshot_t.valid = false
imu_snapshot_t.calibrated = false
```

After calibration:

```text
imu_snapshot_t.valid = true when attitude update succeeds
imu_snapshot_t.calibrated = true
```
