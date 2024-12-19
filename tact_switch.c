#include "tact_switch.h"

void tact_switch_init() {
    gpio_init(pin);               // Initialize the GPIO pin
    gpio_set_dir(pin, GPIO_IN);   // Set the pin as input
    gpio_pull_up(pin);            // Enable pull-up resistor
}

bool tact_switch_is_pressed() {
    return !gpio_get(pin); // Return true if button is pressed (logic low)
}
