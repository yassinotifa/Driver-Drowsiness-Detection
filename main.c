#include "ir.h"
#include "buzzer.h"
#include "imu.h"
#include "lcd_driver.h"
#include "pico/stdlib.h"
#include "tact_switch.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>

#define TACT_SWITCH_PIN 20  // Update this to your actual pin
#define UART_TX_PIN 0       // TX Pin
#define UART_RX_PIN 1       // RX Pin
#define TILT_THRESHOLD 50.0  // Tilt threshold in degrees
#define TILT_DURATION 3000000  // 3 seconds in microseconds
#define BUTTON_RESPONSE_DURATION 3000000  // 3 seconds in microseconds
#define SNOOZE_DURATION 5000000  // 3 seconds in microseconds
#define EMERGENCY_WAIT_DURATION 3000000  // 3 seconds in microseconds


// UART baud rate
#define BAUD_RATE 9600

// SENDER 

void uart_send_signal(char signal) {
    uart_putc(uart0, signal); 
}


// int main() {
//     // initialize standard I/O
//     stdio_init_all();
//     // initialize buzzer 
//     buzzer_init();
    
//     // initialize IMU
//     ir_init();
//     imu_init();
//     sleep_ms(100); // Delay for 100ms to allow IMU initialization

//     // Set up UART
//     uart_init(uart0, BAUD_RATE);
//     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
//     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

//     // IMU variables
//     float tilt_x, tilt_y, tilt_z;
//     float initialx, initialy, initialz;
//     uint32_t tilt_start_time = 0;  // Tracks when the tilt condition starts
//     bool tilt_detected = false;
//     imu_get_tilt(&tilt_x, &tilt_y, &tilt_z);
//     initialx = tilt_x;
//     initialy = tilt_y;
//     initialz = tilt_z;

//     // IR variables
//     uint32_t eye_closed_time = 0;
//     uint32_t buzzer_start_time = 0;
//     const uint32_t threshold_time = 3000000;  // 5 seconds in microseconds
//     const uint32_t extra_time = 3000000;      // 3 additional seconds in microseconds
//     bool sleeping = false;
//     bool signal_sent = false;

//     //bool parallel_motor = false;
    
//     while (true) {
//         while (uart_is_readable(uart0)) {
//             char received = uart_getc(uart0);
//             if (received == 'B') {
//                 buzzer_on();
//             }
//             if (received == 'X') {
//                 buzzer_off();
//             }
//         }
//         // Get tilt angles
//         imu_get_tilt(&tilt_x, &tilt_y, &tilt_z);
    
//         // Tilt handling
//         if((tilt_x - initialx >= 50) || (tilt_x - initialx <= -50) || (tilt_y - initialy >= 50) || (tilt_y - initialy <= -50) || (tilt_z - initialz >= 50) || (tilt_z - initialz <= -50)){
//             if (!tilt_detected) {
//                 tilt_start_time = time_us_32();  // Record the start time of tilt
//                 tilt_detected = true;
//             } else if (time_us_32() - tilt_start_time >= TILT_DURATION) {
//                 uart_send_signal('S');  // Send the signal if tilt persists for 3 seconds
//             }
//         } else if (!sleeping){
//             tilt_detected = false;  // Reset tilt detection
//             tilt_start_time = 0;   // Reset the timer
//             uart_send_signal('Y');
//         }

//         // Blinkink handling
//         if (!ir_read()) {  // Assuming 0 means eye is closed
//             if (eye_closed_time == 0) {
//                 eye_closed_time = time_us_32();  // Start the timer
//             } else if (time_us_32() - eye_closed_time > threshold_time) {
//                 sleeping = true;
//                 if (buzzer_start_time == 0) {
//                     buzzer_start_time = time_us_32();  // Start the buzzer timer
//                 }
//             }
//         } else {
//             sleeping = false;
//             eye_closed_time = 0;
//             buzzer_start_time = 0;
//         }

//         if (sleeping) {
//             buzzer_on();
//             // Wait for an additional 3 seconds after the buzzer starts
//             if (time_us_32() - buzzer_start_time > extra_time && !signal_sent && !tilt_detected) {
//                 uart_send_signal('1');  
//                 signal_sent = true;
//             }
//         } else if(!tilt_detected){
//             buzzer_off();
//             uart_send_signal('2');
//             signal_sent = false;
//         }

//     }
//     return 0;
// }

//RECIEVER 

void uart_init_receiver() {
    uart_init(uart0, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Define stop_motor here
volatile bool stop_motor = false;

int main() {
    // Initialize UART
    uart_init(uart0, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initialize LCD
    spi_inst_t *spi = spi0;
    lcd_init(spi);
    backlight_init();
    lcd_clear(spi);
    lcd_set_cursor(spi0, 0, 0);

    char buffer[64];
    int index = 0;

    bool message_displayed = false;
    uint32_t display_start_time;  // Tracks when the tilt condition starts

    // tact switch handling
    tact_switch_init(TACT_SWITCH_PIN);
    bool snooze = false;
    uint32_t snooze_start_time;  // Tracks when the tilt condition starts

    // MOTOR
    motor_driver_init();
    bool motor = false;
    uint32_t motor_start_time;  // Tracks when the tilt condition starts
    bool check = false;
    bool check2 = false;

    // State tracking for motor actions
    static bool motor_shaking = false;
    static float ere = 0.05;              // Motor speed
    static int shake_step = 0;           // Shake step tracker
    static uint32_t last_action_time = 0; // Timing for shaking
    bool rotate_first = false;

    bool snooze_flag = false;
    static uint32_t post_snooze_start_time;

    bool exit_snooze = false;

    while (true) {
        // Read data from UART
        while (uart_is_readable(uart0)) {
            char received = uart_getc(uart0);
            if (received == 'S' && !message_displayed && !snooze) {
                lcd_clear(spi);
                lcd_set_cursor(spi, 0, 0);
                display_start_time = time_us_32();  
                lcd_display_string(spi0, "ARE YOU AWAKE");
                message_displayed = true;
                exit_snooze = false;
            }
            else if (received == 'Y') {
                if (snooze){
                    snooze_start_time = 0;  
                    lcd_display_string(spi0, "EYES ON THE ROAD");
                    sleep_ms(500);
                    lcd_clear(spi);
                    snooze = false;
                }
                else if (motor_shaking){
                    stop_motor = true;   // Set stop flag
                    motor_shaking = false;
                    motor_stop();        // Ensure motor stops
                    lcd_clear(spi);
                    message_displayed = false;
                    //display_start_time = 0;
                    snooze = false;
                    //snooze_start_time = 0; 
                    motor = false;
                    //motor_start_time = 0;
                    check = false;
                    check2 = false;
                    motor_shaking = false;
                    ere = 0.1;              // Motor speed
                    shake_step = 0;           // Shake step tracker
                    last_action_time = 0; // Timing for shaking
                    rotate_first = false;
                    uart_send_signal('X');
                }
                else if(message_displayed && !exit_snooze){
                    lcd_clear(spi);  // Clear the LCD screen
                    lcd_set_cursor(spi, 0, 0);
                    //message_displayed = false;
                    lcd_display_string(spi0, "EYES ON THE ROAD");
                    sleep_ms(500);
                    lcd_clear(spi);
                    //display_start_time = 0;  
                    uart_send_signal('X');  
                    exit_snooze = true; 
                    message_displayed = false;
                    snooze = false;
                    //snooze_start_time = 0; 
                    motor = false;
                    //motor_start_time = 0;
                    check = false;
                    check2 = false;
                    motor_shaking = false;
                    ere = 0.05;              // Motor speed
                    shake_step = 0;           // Shake step tracker
                    last_action_time = 0; // Timing for shaking
                    rotate_first = false;
                }
            }
            if (received == '1' && !motor_shaking) {
                stop_motor = false;  // Reset stop flag
                motor_shaking = true;
                ere = 0.05;           // Reset speed
                shake_step = 0;      // Reset shake steps
                // EYES CLOSED SO NO MESSAGE
                lcd_clear(spi);  // Clear the LCD screen
                lcd_set_cursor(spi, 0, 0);
                message_displayed = false;
                snooze = false;
                } else if (received == '2') {
                    stop_motor = true;   // Set stop flag
                    motor_shaking = false;
                    motor_stop();        // Ensure motor stops
                }
        }
        // Check if the button is pressed
        if (message_displayed && tact_switch_is_pressed(TACT_SWITCH_PIN)) {
            lcd_clear(spi);  // Clear the LCD screen
            lcd_set_cursor(spi, 0, 0);
            message_displayed = false;
            display_start_time = 0;  
            uart_send_signal('X');  
            // ENTER SNOOZE STATE 
            snooze = true;
            snooze_start_time = time_us_32(); 
            // motor reset
            // motor_start_time = 0;  
            // motor = false;
            snooze_flag = true;
            post_snooze_start_time = time_us_32();
        }
        if (time_us_32() - display_start_time >= BUTTON_RESPONSE_DURATION && message_displayed) {
            if (!check && !check2){
                motor_start_time = time_us_32();  
                check = true;
            }
            else if (check == true){
                check2 = true;
            }
            //motor = true;
            uart_send_signal('B');  // Send the signal if tilt persists for 3 seconds
        }
        if (time_us_32() - snooze_start_time >= SNOOZE_DURATION && snooze){
            snooze = false;
            snooze_start_time = 0;
            lcd_clear(spi);
            lcd_set_cursor(spi, 0, 0);
            display_start_time = time_us_32();  
            lcd_display_string(spi0, "ARE YOU AWAKE");
            message_displayed = true;
            //snooze_flag = false;
        }
        if (time_us_32() - motor_start_time >= EMERGENCY_WAIT_DURATION && check2){
            //  motor_rotate_right(0.5);
            if (!rotate_first && (snooze == false) && !snooze_flag){
                stop_motor = false;  // Reset stop flag
                motor_shaking = true;
                ere = 0.05;           // Reset speed
                shake_step = 0;      // Reset shake steps
                rotate_first = true;
            }
        }
        if (snooze_flag) {
            //static uint32_t post_snooze_start_time = 0;
            if (time_us_32() - post_snooze_start_time >= 11000000) {  // 3-second delay
                //uart_send_signal('B');  // Activate buzzer
                //post_snooze_start_time = time_us_32();  // Reset timer for next stage
                snooze_flag = false;  // Move to motor stage
            } 
        }
        // Handle motor shaking
        if (motor_shaking && !stop_motor) {
            lcd_clear(spi);  // Clear the LCD screen
            lcd_set_cursor(spi, 0, 0);
            message_displayed = false;
            display_start_time = 0;  
            // Check if enough time has passed for the next action
            if (time_us_32() - last_action_time >= 2000000) { // 2-second delay
                last_action_time = time_us_32();

                // Perform the next shake step
                if (shake_step % 2 == 0) {
                    motor_rotate_right(ere);
                } else {
                    motor_rotate_left(ere);
                    // Increment speed after each pair of rotations
                    if (shake_step % 2 == 1 && ere < 1.0) {
                        ere += 0.025;
                    }
                }

                // Update shake state
                shake_step++;
                if (shake_step >= 10000) { // Stop after 5 full cycles (10 steps)
                    motor_stop();
                    motor_shaking = false;
                }
            }
        }
        // Small delay to prevent busy waiting
        sleep_ms(50);
    }
    return 0;
}