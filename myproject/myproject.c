#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "asm330lhh-pid/asm330lhh_reg.h"


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define INT1_PIN 15
#define INT2_PIN 16

void interrupt1_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        // uart_puts(UART_ID, "Got an interrupt!\n");
        printf("Interrupt 1 Handler\n");
    }
}
void interrupt2_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        // uart_puts(UART_ID, "Got an interrupt!\n");
        printf("Interrupt 2 Handler\n");
    }
}

void asm330lhh_spi_init(void) {
    spi_init(SPI_PORT, 1000*1000); // 1 MHz, enable SPI hardware first
    spi_set_format(SPI_PORT, 
                    8,              // 8 bits per word
                    SPI_CPHA_1,     // clock idle high
                    SPI_CPOL_1,     // sample on second edge
                    SPI_MSB_FIRST); // MSB first
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    spi_set_slave(SPI_PORT, false); // Set as master
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

int32_t pico_asm330lhh_read_reg(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    // Set the CS pin low to start communication
    gpio_put(PIN_CS, 0);
    // Send the register address with the read bit set
    spi_write_read_blocking(SPI_PORT, &reg, data, len);
    // Set the CS pin high to end communication
    gpio_put(PIN_CS, 1);
    return 0; // Return success
}

int32_t pico_asm330lhh_write_reg(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    // Set the CS pin low to start communication
    gpio_put(PIN_CS, 0);
    // Send the register address with the write bit set
    spi_write_blocking(SPI_PORT, &reg, 1);
    // Write the data to the register
    spi_write_blocking(SPI_PORT, data, len);
    // Set the CS pin high to end communication
    gpio_put(PIN_CS, 1);
    return 0; // Return success
}

int main()
{
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    asm330lhh_spi_init();
    printf("ASM330LHH SPI Initialized\n");

    stmdev_ctx_t dev_ctx = {
        .write_reg = pico_asm330lhh_write_reg,
        .read_reg = pico_asm330lhh_read_reg,
        .handle = NULL
    };

    uint8_t whoami = 0x66;
    asm330lhh_device_id_get(&dev_ctx, &whoami);
    printf("Device ID: 0x%02X\n", whoami);

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_puts(UART_ID, " Hello, UART!\n");
    

    // Initilize interrupt pins
    // We will use GPIO 15 and 16 for interrupts
    gpio_init(INT1_PIN);
    gpio_init(INT2_PIN);
    gpio_set_dir(INT1_PIN, GPIO_IN);
    gpio_set_dir(INT2_PIN, GPIO_IN);
    gpio_pull_down(INT1_PIN);
    gpio_pull_down(INT2_PIN);

    gpio_set_irq_enabled_with_callback(INT1_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true, 
        &interrupt1_handler);
    gpio_set_irq_enabled_with_callback(INT2_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true, 
        &interrupt2_handler);
    gpio_set_irq_enabled(INT1_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true);
    gpio_set_irq_enabled(INT2_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true);
    
    while (true) {
    }
}
