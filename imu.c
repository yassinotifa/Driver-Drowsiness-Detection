// #include "imu.h"
// #include "hardware/i2c.h"
// #include "hardware/gpio.h"
// #include <stdio.h>
// #include <math.h>

// // MPU6050 I2C address
// #define MPU6050_ADDR 0x68

// // MPU6050 registers
// #define MPU6050_ACCEL_XOUT_H 0x3B
// #define MPU6050_GYRO_XOUT_H  0x43
// #define MPU6050_PWR_MGMT_1   0x6B
// #define MPU6050_WHO_AM_I     0x75

// // Variables to store accelerometer data
// float Ax = 0, Ay = 0, Az = 0;

// // Variables to store gyroscope data
// float Gx = 0, Gy = 0, Gz = 0;

// // Helper function to read data from a register
// static int16_t readRegister16(i2c_inst_t *i2c, uint8_t address, uint8_t reg) {
//     uint8_t buffer[2];
//     if (i2c_write_blocking(i2c, address, &reg, 1, true) != 1) {
//         printf("Failed to write to register 0x%02X\n", reg);
//         return 0;
//     }
//     if (i2c_read_blocking(i2c, address, buffer, 2, false) != 2) {
//         printf("Failed to read from register 0x%02X\n", reg);
//         return 0;
//     }
//     return (buffer[0] << 8) | buffer[1];
// }

// // Initialize the IMU
// void initializeIMU() {
//     // Initialize I2C at 400kHz
//     if (i2c_init(i2c0, 400 * 1000) == 0) {
//         printf("Failed to initialize I2C\n");
//         return;
//     }
    
//     // Configure GPIO pins for I2C
//     gpio_set_function(4, GPIO_FUNC_I2C);  // SDA pin
//     gpio_set_function(5, GPIO_FUNC_I2C);  // SCL pin
//     gpio_pull_up(4);
//     gpio_pull_up(5);

//     // Wake up the MPU6050
//     uint8_t buf[2] = {MPU6050_PWR_MGMT_1, 0x00};
//     if (i2c_write_blocking(i2c0, MPU6050_ADDR, buf, 2, false) != 2) {
//         printf("Failed to wake up MPU6050\n");
//         return;
//     }

//     // Check the WHO_AM_I register
//     uint8_t who_am_i;
//     if (i2c_write_blocking(i2c0, MPU6050_ADDR, (uint8_t[]){MPU6050_WHO_AM_I}, 1, true)) {
//         printf("Failed to write WHO_AM_I register\n");
//         return;
//     }
//     if (i2c_read_blocking(i2c0, MPU6050_ADDR, &who_am_i, 1, false) != 1) {
//         printf("Failed to read WHO_AM_I register\n");
//         return;
//     }

//     if (who_am_i != 0x68) {
//         printf("Failed to initialize MPU6050! WHO_AM_I = 0x%02X\n", who_am_i);
//     } else {
//         printf("MPU6050 initialized successfully!\n");
//     }
// }

// // Read accelerometer data
// void readAccelerometerData() {
//     Ax = readRegister16(i2c0, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H) / 16384.0;  // Divide by sensitivity scale factor
//     Ay = readRegister16(i2c0, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H + 2) / 16384.0;
//     Az = readRegister16(i2c0, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H + 4) / 16384.0;

//     printf("Accelerometer: Ax = %.2f, Ay = %.2f, Az = %.2f\n", Ax, Ay, Az);
// }

// // Read gyroscope data
// void readGyroscopeData() {
//     Gx = readRegister16(i2c0, MPU6050_ADDR, MPU6050_GYRO_XOUT_H) / 131.0;  // Divide by sensitivity scale factor
//     Gy = readRegister16(i2c0, MPU6050_ADDR, MPU6050_GYRO_XOUT_H + 2) / 131.0;
//     Gz = readRegister16(i2c0, MPU6050_ADDR, MPU6050_GYRO_XOUT_H + 4) / 131.0;

//     printf("Gyroscope: Gx = %.2f, Gy = %.2f, Gz = %.2f\n", Gx, Gy, Gz);
// }

#include "imu.h"
#include <math.h>
#include "pico/stdlib.h"

// Helper function to write to a register
static void write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(IMU_I2C, LSM6DSOX_ADDR, buffer, 2, false);
}

// Helper function to read multiple bytes from a register
static void read_register(uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(IMU_I2C, LSM6DSOX_ADDR, &reg, 1, true);
    i2c_read_blocking(IMU_I2C, LSM6DSOX_ADDR, buffer, length, false);
}

// Initialize the IMU
void imu_init() {
    // Initialize I2C
    i2c_init(IMU_I2C, 400000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    // Configure accelerometer to 2g, 104 Hz
    write_register(LSM6DSOX_CTRL1_XL, 0x50);

    // Configure gyroscope to 250 dps, 104 Hz
    write_register(LSM6DSOX_CTRL2_G, 0x50);
}

// Read accelerometer data
bool imu_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buffer[6];
    read_register(LSM6DSOX_OUTX_L_XL, buffer, 6);

    if (buffer) {
        *ax = (int16_t)((buffer[1] << 8) | buffer[0]);
        *ay = (int16_t)((buffer[3] << 8) | buffer[2]);
        *az = (int16_t)((buffer[5] << 8) | buffer[4]);
        return true;
    }
    return false;
}

// Read gyroscope data
bool imu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buffer[6];
    read_register(LSM6DSOX_OUTX_L_G, buffer, 6);

    if (buffer) {
        *gx = (int16_t)((buffer[1] << 8) | buffer[0]);
        *gy = (int16_t)((buffer[3] << 8) | buffer[2]);
        *gz = (int16_t)((buffer[5] << 8) | buffer[4]);
        return true;
    }
    return false;
}

// Compute tilt angles (pitch and roll) from accelerometer data
// void imu_get_tilt(float *pitch, float *roll) {
//     int16_t ax, ay, az;
//     if (imu_read_accel(&ax, &ay, &az)) {
//         float ax_g = ax / 16384.0f; // Convert to g
//         float ay_g = ay / 16384.0f;
//         float az_g = az / 16384.0f;

//         *pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI);
//         *roll = atan2f(ay_g, az_g) * (180.0f / M_PI);
//     }
// }

// Compute tilt angles for x, y, z axes
void imu_get_tilt(float *tilt_x, float *tilt_y, float *tilt_z) {
    int16_t ax, ay, az;
    if (imu_read_accel(&ax, &ay, &az)) {
        float ax_g = ax / 16384.0f; // Convert to g
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;

        float magnitude = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

        *tilt_x = acosf(ax_g / magnitude) * (180.0f / M_PI);
        *tilt_y = acosf(ay_g / magnitude) * (180.0f / M_PI);
        *tilt_z = acosf(az_g / magnitude) * (180.0f / M_PI);
    }
}