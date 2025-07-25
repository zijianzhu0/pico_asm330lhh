#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/time.h>
#include <pico/types.h>
#include "pico/multicore.h"
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "asm330lhh-pid/asm330lhh_reg.h"


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define INT1_PIN 15
#define INT2_PIN 16

#define PACKET_START 0xAA
#define TYPE_ACCEL 0x01
#define TYPE_GYRO  0x02

static uint8_t whoamI, rst;
static float ts_res = 0.000025; // uncalibrated ts_res is 25us
static double timestamp_ms; //used for printout
static uint32_t timestamp;

static bool int1_ready = false;
static bool int2_ready = false;

static uint32_t int1_count = 0;
static uint32_t int2_count = 0;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];

static float acceleration_mg[3];
static float angular_rate_mdps[3];

void interrupt1_handler(uint gpio, uint32_t events);
void interrupt2_handler(uint gpio, uint32_t events);

void flash_led(int times, int duration_ms);
typedef struct {
    uint8_t start;
    uint8_t type;
    uint32_t timestamp;
    float data[3];  // X, Y, Z for accel/gyro; [0] for temp
    uint8_t checksum;
} __attribute__((packed)) Packet;  // Ensure packed structure\

uint8_t calculate_checksum(Packet *pkt);

void asm330lhh_spi_init(void);
void asm330lhh_init(stmdev_ctx_t *ctx);

int32_t pico_asm330lhh_read_reg(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    reg = reg | 0x80; // Set the MSB for read operation
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &reg, 1); // Send register address
    spi_read_blocking(SPI_PORT, 0, data, len); // Read data
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

stmdev_ctx_t dev_ctx = {
        .write_reg = pico_asm330lhh_write_reg,
        .read_reg = pico_asm330lhh_read_reg,
        .handle = NULL
    };


void core1_entry() {
    gpio_set_irq_enabled_with_callback(INT2_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true, 
        &interrupt2_handler);
    while (true) {
        sleep_ms(1000);
        printf("Core 1 alive!  INT1 count: %d, INT2 count: %d\n", int1_count, int2_count);
        flash_led(2, 100);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(500);
    // if (cyw43_arch_init()) {
    //     printf("Wi-Fi init failed");
    //     return -1;
    // }
    // while (!stdio_usb_connected()) {
    //     sleep_ms(10);
    // }

    asm330lhh_init(&dev_ctx);
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
    gpio_disable_pulls(INT1_PIN);
    // gpio_pull_down(INT2_PIN); asm330lhh INT2 pin is open drain, do not pull up or down
    gpio_disable_pulls(INT2_PIN);

    gpio_set_irq_enabled_with_callback(INT1_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true, 
        &interrupt1_handler);
    
    // Launch core 1 to handle INT2
    multicore_launch_core1(core1_entry);

    while (true) {
        bool int1_pin_state = gpio_get(INT1_PIN);
        bool int2_pin_state = gpio_get(INT2_PIN);
        if (int1_ready | int1_pin_state) {
            data_raw_acceleration[0] = 0;
            data_raw_acceleration[1] = 0;
            data_raw_acceleration[2] = 0;
            timestamp = 0;
            asm330lhh_timestamp_raw_get(&dev_ctx, &timestamp);
            asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            acceleration_mg[0] =
                asm330lhh_from_fs2g_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] =
                asm330lhh_from_fs2g_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] =
                asm330lhh_from_fs2g_to_mg(data_raw_acceleration[2]);
            Packet accel_pkt = {
                PACKET_START,
                TYPE_ACCEL,
                timestamp,
                {
                    acceleration_mg[0],
                    acceleration_mg[1],
                    acceleration_mg[2]
                },
                0U
            };
            accel_pkt.checksum = calculate_checksum(&accel_pkt);
            uart_write_blocking(UART_ID, (uint8_t *)&accel_pkt, sizeof(accel_pkt));
            int1_ready = false; // Reset the flag
        }
        if (int2_ready | int2_pin_state) {
            data_raw_angular_rate[0] = 0;
            data_raw_angular_rate[1] = 0;
            data_raw_angular_rate[2] = 0;
            timestamp = 0;
            asm330lhh_timestamp_raw_get(&dev_ctx, &timestamp);
            asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
            angular_rate_mdps[0] =
            asm330lhh_from_fs250dps_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] =
            asm330lhh_from_fs250dps_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] =
            asm330lhh_from_fs250dps_to_mdps(data_raw_angular_rate[2]);
            Packet gyro_pkt = {
                PACKET_START,
                TYPE_GYRO,
                timestamp,
                {
                    angular_rate_mdps[0],
                    angular_rate_mdps[1],
                    angular_rate_mdps[2]
                },
                0U
            };
            gyro_pkt.checksum = calculate_checksum(&gyro_pkt);
            uart_write_blocking(UART_ID, (uint8_t *)&gyro_pkt, sizeof(gyro_pkt));
            int2_ready = false; // Reset the flag
        }
    }
}

void interrupt1_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        int1_ready = true; // Set the flag to indicate interrupt 1 was triggered
        // int16_t data_raw_acceleration[3] = {0};
        // asm330lhh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
        int1_count++;
    }
}
void interrupt2_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        int2_ready = true; // Set the flag to indicate interrupt 2 was triggered
        // int16_t data_raw_angular_rate[3] = {0};
        // asm330lhh_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
        int2_count++;
    }
}

void flash_led(int times, int duration_ms) {
    for (int i = 0; i < times; i++) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);  
        sleep_ms(duration_ms);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(duration_ms);
    }
}

// XOR Checksum calculation
uint8_t calculate_checksum(Packet *pkt) {
    uint8_t *bytes = (uint8_t *)pkt;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(Packet) - 1; i++) {
        checksum ^= bytes[i];  // XOR all bytes except start and checksum
    }
    return checksum;
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
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

void asm330lhh_init(stmdev_ctx_t *ctx) {
    // This function is a placeholder for device configuration
    // In this case, we assume the device is already configured
    // You can add specific configuration code here if needed
    asm330lhh_spi_init();
    printf("ASM330LHH SPI Initialized\n");

    asm330lhh_device_id_get(&dev_ctx, &whoamI);
    printf("Device ID: 0x%02X\n", whoamI);
    
    if (whoamI != ASM330LHH_ID)
	  while (1)

	/* Restore default configuration */
	asm330lhh_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
	  asm330lhh_reset_get(&dev_ctx, &rst);
	} while (rst);
	/* Start device configuration. */
	asm330lhh_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
	/* Enable Block Data Update */
	asm330lhh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	/* Set Output Data Rate */
	asm330lhh_xl_data_rate_set(&dev_ctx, ASM330LHH_XL_ODR_104Hz);
	asm330lhh_gy_data_rate_set(&dev_ctx, ASM330LHH_GY_ODR_104Hz);
	/* Set full scale */
	asm330lhh_xl_full_scale_set(&dev_ctx, ASM330LHH_2g);
	asm330lhh_gy_full_scale_set(&dev_ctx, ASM330LHH_250dps);
	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(&dev_ctx, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	// enable timestamping
    asm330lhh_timestamp_set(&dev_ctx, PROPERTY_ENABLE);
    asm330lhh_get_ts_res(&dev_ctx, &ts_res);

    // enable interrupts on INT1 and INT2 pins
    asm330lhh_pin_int1_route_t int1_route={0};
    asm330lhh_pin_int2_route_t int2_route={0};
    int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
    asm330lhh_pin_int1_route_set(&dev_ctx, &int1_route);
    asm330lhh_pin_int2_route_set(&dev_ctx, &int2_route);

    // Directly enable INT1_DRDY_XL and INT2_DRDY_G via register write
    uint8_t int1_ctrl = 0;
    uint8_t int2_ctrl = 0;
    int1_ctrl |= (1 << 0); // INT1_DRDY_XL is bit 0
    int2_ctrl |= (1 << 1); // INT2_DRDY_G is bit 1
    pico_asm330lhh_write_reg(NULL, 0x0D, &int1_ctrl, 1); // 0x0D = INT1_CTRL
    pico_asm330lhh_write_reg(NULL, 0x0E, &int2_ctrl, 1); // 0x0E = INT2_CTRL

}