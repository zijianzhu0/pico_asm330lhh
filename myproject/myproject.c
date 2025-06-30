#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <stdbool.h>
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
int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // SPI initialisation. This example will use SPI at 1MHz.
    
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
    // Set the SPI baud rate to 1MHz
    spi_init(SPI_PORT, 1000*1000);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Set up our UART
    uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Use some the various UART functions to send out data
    // In a default system, printf will also output via the default UART
    
    // Send out a string, with CR/LF conversions
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
