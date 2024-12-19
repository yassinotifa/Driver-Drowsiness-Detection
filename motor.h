#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Motor control pin definitions
#define ENA 16
#define M1 29
#define M2 28
//#define LPWM 29

// PWM frequency (in Hz)
#define PWM_FREQUENCY 25000  // Increase to 10 kHz
#define PWM_WRAP_VALUE 65535  // Max value for the duty cycle (16-bit)

/**
 * @brief Initialize the motor driver.
 * Sets up GPIO and PWM configurations for controlling the motor.
 */
void motor_driver_init(void);

/**
 * @brief Set the duty cycle for a specific PWM pin.
 * 
 * @param gpio GPIO pin configured for PWM.
 * @param duty_cycle Duty cycle value (0 to wrap value).
 */

//void set_pwm_duty(uint gpio, uint16_t duty_cycle);

void set_pwm_duty(uint gpio, uint16_t duty_cycle);

/**
 * @brief Rotate the motor to the left.
 * Enables the left direction and sets the appropriate PWM duty cycle.
 */
void motor_rotate_left(float percentage);

/**
 * @brief Rotate the motor to the right.
 * Enables the right direction and sets the appropriate PWM duty cycle.
 */
void motor_rotate_right(float percentage);

/**
 * @brief Stop the motor.
 * Disables both directions and sets the PWM duty cycle to 0.
 */
void motor_stop(void);
// void motor_shake(void);

// Declare stop_motor as an external variable
// extern volatile bool stop_motor;

#endif // MOTOR_DRIVER_H
