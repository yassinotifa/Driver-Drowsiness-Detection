// #ifndef IMU_H
// #define IMU_H

// #include <stdint.h>
// #include <stdbool.h>

// // Variables to store accelerometer data
// extern float Ax, Ay, Az;

// // Variables to store gyroscope data
// extern float Gx, Gy, Gz;

// // Function prototypes
// void initializeIMU();
// void readAccelerometerData();
// void readGyroscopeData();

// #endif // IMU_H

#ifndef IMU_H
#define IMU_H

#include "hardware/i2c.h"

// I2C Pins
#define IMU_I2C i2c0
#define IMU_SDA_PIN 12
#define IMU_SCL_PIN 13

// LSM6DSOX Register Addresses
#define LSM6DSOX_ADDR        0x6A // I2C address
#define LSM6DSOX_CTRL1_XL    0x10 // Accelerometer control register
#define LSM6DSOX_CTRL2_G     0x11 // Gyroscope control register
#define LSM6DSOX_OUTX_L_XL   0x28 // Accelerometer data register (low byte)
#define LSM6DSOX_OUTX_L_G    0x22 // Gyroscope data register (low byte)

// Functions
void imu_init();
bool imu_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
bool imu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
// void imu_get_tilt(float *pitch, float *roll);
void imu_get_tilt(float *tilt_x, float *tilt_y, float *tilt_z);

#endif // IMU_H
