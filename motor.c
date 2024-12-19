#include "pico/stdlib.h"
#include "motor.h"
// #include "pico/platform.h"  // Include the correct header for clock access
#include "hardware/clocks.h"


static uint pwm_slice_ena;

void motor_driver_init() {
    // Initialize GPIO for direction control (M1, M2)
    gpio_init(M1);
    gpio_set_dir(M1, GPIO_OUT);
    gpio_put(M1, 0);  // Set to default direction

    gpio_init(M2);
    gpio_set_dir(M2, GPIO_OUT);
    gpio_put(M2, 0);  // Set to default direction

    // Initialize PWM for ENA (Pin 12)
    gpio_set_function(ENA, GPIO_FUNC_PWM);  // Set ENA to PWM function
    pwm_slice_ena = pwm_gpio_to_slice_num(ENA);  // Get PWM slice number for ENA

    // Calculate the clock divider for the desired PWM frequency
    float pwm_freq = PWM_FREQUENCY;
    float clkdiv = (float)clock_get_hz(clk_sys) / (PWM_FREQUENCY * PWM_WRAP_VALUE);  // Calculate divider
    pwm_set_clkdiv(pwm_slice_ena, clkdiv);           // Set the clock divider for PWM

    pwm_set_wrap(pwm_slice_ena, PWM_WRAP_VALUE);  // Set the max value for PWM
    pwm_set_enabled(pwm_slice_ena, true);         // Enable PWM on ENA
}

// Function to set the PWM duty cycle for ENA (motor speed control)
void set_pwm_duty(uint gpio, uint16_t duty_cycle) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_gpio_level(gpio, duty_cycle); // Set PWM duty cycle
}

// Function to rotate motor left at 100% speed
void motor_rotate_left(float percentage) {
    gpio_put(M1, 0);  // Disable right direction
    gpio_put(M2, 1);  // Enable left direction

    // Set the PWM duty cycle for motor speed (100% speed)
    set_pwm_duty(ENA, PWM_WRAP_VALUE * percentage); // 100% duty cycle for max speed
}

// Function to rotate motor right at 100% speed
void motor_rotate_right(float percentage) {
    gpio_put(M1, 1);  // Enable right direction
    gpio_put(M2, 0);  // Disable left direction

    // Set the PWM duty cycle for motor speed (100% speed)
    set_pwm_duty(ENA, PWM_WRAP_VALUE * percentage); // 100% duty cycle for max speed
}

// Function to stop the motor
void motor_stop() {
    gpio_put(M1, 0);
    gpio_put(M2, 0);

    // Stop PWM (set duty cycle to 0)
    set_pwm_duty(ENA, 0);
}

// volatile bool stop_motor = false;  // Global flag to interrupt motor shaking

// void motor_shake() {
//     float ere = 0.1;  // Initialize 'ere' to 0.1

//     while (true) {
//         if (stop_motor) break;     // Check if stop signal was received
//         motor_rotate_right(ere);  // Use 'ere' for motor speed
//         sleep_ms(2000);
//         if (stop_motor) break;     // Check if stop signal was received
//         motor_rotate_left(ere);   // Use 'ere' for motor speed in the opposite direction
//         sleep_ms(2000);
//         if (stop_motor) break;     // Check if stop signal was received
        
//         if (ere < 1.0){
//             ere += 0.1;  // Increment 'ere' by 0.1 in each iteration
//         }
//     }
//         motor_stop();                  // Ensure motor stops
// }

