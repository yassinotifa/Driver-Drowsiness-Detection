#ifndef TACT_SWITCH_H
#define TACT_SWITCH_H

#include "pico/stdlib.h"
#define pin 21

// Function prototypes
/**
 * @brief Initialize the tact switch GPIO pin.
 *
 * @param pin The GPIO pin connected to the tact switch.
 */
void tact_switch_init();

/**
 * @brief Check the state of the tact switch.
 *
 * @param pin The GPIO pin connected to the tact switch.
 * @return true if the button is pressed, false otherwise.
 */
bool tact_switch_is_pressed();

#endif // TACT_SWITCH_H
