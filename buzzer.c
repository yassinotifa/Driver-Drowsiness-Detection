#include "buzzer.h"
#include "pico/stdlib.h"

#define BUZZER_PIN 25  // GPIO pin 6 for buzzer
// #define TEST_PIN 4  // GPIO pin 6 for buzzer


void buzzer_init(void) {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);  // Set buzzer pin as output
    // gpio_init(TEST_PIN);
    // gpio_set_dir(TEST_PIN, GPIO_OUT);  // Set buzzer pin as output
    // gpio_put(TEST_PIN, 1);  // Turns the buzzer on
}

void buzzer_on(void) {
    gpio_put(BUZZER_PIN, 1);  // Turns the buzzer on
}

void buzzer_off(void) {
    gpio_put(BUZZER_PIN, 0);  // Turns the buzzer off
}
