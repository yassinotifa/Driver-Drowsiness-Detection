#include "ir.h"
#include "pico/stdlib.h"

#define IR_PIN 27 // Adjust according to your setup

void ir_init(void) {
    gpio_init(IR_PIN);
    gpio_set_dir(IR_PIN, GPIO_IN);  // Set IR pin as input
}

uint8_t ir_read(void) {
    return gpio_get(IR_PIN);  // Reads the digital signal from the IR sensor
}
