#include "font.h"
#include "lcd_driver.h"
#include "pico/stdlib.h"


// Pin Definitions
#define sck_pin 6    // Clock
#define mosi_pin 7  // Data input (DIN)
#define dc_pin 17    // DataCommand
#define ce_pin 18    // Chip Enable
#define rst_pin 15   // Reset
#define BACKLIGHT_PIN 19  // Replace with your GPIO pin


#define LCD_CMD 0    // Command mode
#define LCD_DATA 1   // Data mode

void backlight_init() {
    gpio_init(BACKLIGHT_PIN);
    gpio_set_dir(BACKLIGHT_PIN, GPIO_OUT);
    gpio_put(BACKLIGHT_PIN, 1);  // Turn backlight on
}

void lcd_send(spi_inst_t *spi, uint8_t type, const uint8_t *data, size_t length) {
    gpio_put(dc_pin, type);
    gpio_put(ce_pin, 0);
    spi_write_blocking(spi, data, length);
    gpio_put(ce_pin, 1);
}

void lcd_init(spi_inst_t *spi) {
    spi_init(spi, 4 * 1000 * 1000); // 4 MHz
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_init(dc_pin);
    gpio_set_dir(dc_pin, GPIO_OUT);
    gpio_init(ce_pin);
    gpio_set_dir(ce_pin, GPIO_OUT);
    gpio_init(rst_pin);
    gpio_set_dir(rst_pin, GPIO_OUT);

    gpio_put(rst_pin, 0);
    sleep_ms(10);
    gpio_put(rst_pin, 1);

    // Example LCD Initialization Sequence
    // uint8_t init_cmds[] = {0x21, 0xB0, 0x04, 0x14, 0x20, 0x0C};
    uint8_t init_cmds[] = {0x21, 0xB8, 0x04, 0x14, 0x20, 0x0C}; // Increased contrast

    lcd_send(spi,LCD_CMD, init_cmds, sizeof(init_cmds));
}


void lcd_clear(spi_inst_t *spi) {
    uint8_t clear_data[504] = {0}; // 504 bytes for a 48x84 LCD (48 rows * 84 columns / 8 bits)
    lcd_send(spi, LCD_DATA, clear_data, sizeof(clear_data));
}

void lcd_set_cursor(spi_inst_t *spi, uint8_t x, uint8_t y) {
    // Set X and Y positions
    uint8_t commands[] = {0x80 | x, 0x40 | y}; // X and Y position commands
    lcd_send(spi,LCD_CMD, commands, sizeof(commands));
}

void lcd_display_string(spi_inst_t *spi, const char *str) {
    while (*str) {
        char c = *str++;
        if (c < 32 || c > 127) {
            c = 32; // Replace unsupported characters with space
        }
        lcd_send(spi, LCD_DATA, font5x8[c - 32], 5);
        uint8_t space = 0x00; // Add a blank column between characters
        lcd_send(spi, LCD_DATA, &space, 1);
    }
}
