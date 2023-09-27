//
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define PIN_CH_MUX_A 2
#define PIN_CH_MUX_B 3
#define PIN_CH_MUX_C 4
#define PIN_CS_MUX_0 5
#define PIN_CS_MUX_1 6

#define NUM_MUX 2

#define NUM_DCO 4

// function declarations
void cv_task();
void dac_init();
void dac_set(uint16_t value);
void dac_task();
void init_pin(uint8_t pin);
void led_init();
void led_task();
void mux_init();

// variables
const uint8_t mux_cs[2] = {PIN_CS_MUX_0, PIN_CS_MUX_1};
const uint8_t mux_bit[3] = {PIN_CH_MUX_A, PIN_CH_MUX_B, PIN_CH_MUX_C};

uint32_t led_start = 0;
uint16_t value = 0;
uint8_t cv_cursor = 0;
uint16_t cv[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//
int main() {
    stdio_init_all();
    dac_init();
    led_init();
    mux_init();

    while (1) {
        cv_task();
        dac_task();
        led_task();
    }
}

void dac_init() {
    spi_init(spi_default, 20 * 1000 * 1000);
    spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
}

void dac_set(uint16_t value) {
    uint16_t meta = 0b0011000000000000;
    uint16_t msg = meta | value;
    spi_write16_blocking(spi_default, &msg, 1);
}

// test function that outputs a saw wave
void dac_task() {
    if (value > 4095) {
        value = 0;
    }

    value += 16;

    dac_set(value);
}

void led_init() {
    init_pin(PICO_DEFAULT_LED_PIN);
    led_start = time_us_32();
}

void led_task() {
    gpio_put(PICO_DEFAULT_LED_PIN, time_us_32() - led_start < 100000 ? 1 : 0);
}

void mux_init() {
    for (int i = 0; i < NUM_MUX; i++) init_pin(mux_cs[i]);
    for (int i = 0; i < 3; i++) init_pin(mux_bit[i]);
}

void init_pin(uint8_t pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void cv_task() {
    // move to the next control voltage
    cv_cursor = cv_cursor == 15 ? 0 : cv_cursor + 1;

    // get the target mux device and device channel
    uint8_t device = cv_cursor / 8;
    uint8_t channel = cv_cursor % 8;

    // deselect all mux devices
    for (int i = 0; i < NUM_MUX; i++) gpio_put(mux_cs[i], 0);

    // set the ic channel select bits
    for (int i = 0; i < 3; i++) gpio_put(mux_bit[i], (channel >> i) & 1);

    // select the target mux device
    gpio_put(mux_cs[device], 1);
}
