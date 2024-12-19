#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "font.h"

// LCD Command/Data mode
#define LCD_CMD  0
#define LCD_DATA 1

// Function prototypes
/**
 * @brief Send a command or data to the LCD.
 *
 * @param spi SPI instance to use.
 * @param dc_pin GPIO pin for Data/Command mode.
 * @param ce_pin GPIO pin for Chip Enable.
 * @param type Type of transmission (LCD_CMD or LCD_DATA).
 * @param data Pointer to the data to be sent.
 * @param length Length of the data.
 */
void lcd_send(spi_inst_t *spi, uint8_t type, const uint8_t *data, size_t length);

/**
 * @brief Initialize the Nokia 5110 LCD.
 *
 * @param spi SPI instance to use.
 * @param sck_pin GPIO pin for SPI clock (SCK).
 * @param mosi_pin GPIO pin for SPI MOSI.
 * @param dc_pin GPIO pin for Data/Command mode.
 * @param ce_pin GPIO pin for Chip Enable.
 * @param rst_pin GPIO pin for Reset.
 */
void lcd_init(spi_inst_t *spi);

/**
 * @brief Clear the LCD screen by writing zeros to all memory locations.
 *
 * @param spi SPI instance to use.
 * @param dc_pin GPIO pin for Data/Command mode.
 * @param ce_pin GPIO pin for Chip Enable.
 */
void lcd_clear(spi_inst_t *spi);

/**
 * @brief Set the cursor position on the LCD.
 *
 * @param spi SPI instance to use.
 * @param dc_pin GPIO pin for Data/Command mode.
 * @param ce_pin GPIO pin for Chip Enable.
 * @param x X position (0-83).
 * @param y Y position (0-5).
 */
void lcd_set_cursor(spi_inst_t *spi, uint8_t x, uint8_t y);

/**
 * @brief Display a string of text on the LCD.
 *
 * @param spi SPI instance to use.
 * @param dc_pin GPIO pin for Data/Command mode.
 * @param ce_pin GPIO pin for Chip Enable.
 * @param str Pointer to the string to be displayed.
 */
void lcd_display_string(spi_inst_t *spi, const char *str);

void backlight_init();

#endif // LCD_DRIVER_H
