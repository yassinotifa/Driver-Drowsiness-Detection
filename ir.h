#ifndef IR_H
#define IR_H

#include <stdint.h>

// Initializes the IR sensor
void ir_init(void);

// Reads the IR sensor value
uint8_t ir_read(void);

#endif
