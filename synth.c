//
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define PIN_CH_MUX_A 2
#define PIN_CH_MUX_B 3
#define PIN_CH_MUX_C 4
#define PIN_CS_MUX_0 5
#define PIN_CS_MUX_1 6
#define LED_PIN PICO_DEFAULT_LED_PIN
#define BLINK_TIME 100000

#define NUM_MUX 2
#define NUM_DCO 4
#define NUM_VOICE 2
#define SAW 0
#define TRI 1
#define SIN 2
#define SQR 3
#define RND 4
#define IDLE 0
#define ATTACK 1
#define DECAY 2
#define SUSTAIN 3
#define RELEASE 4
#define RES 1023

// function declarations
void cv_task();
void dac_init();
void dac_set(uint16_t value);
void dac_task();
void env_task();
uint32_t hz_to_us(float hz);
void init_pin(uint8_t pin);
void led_init();
void led_task();
void lfo_task();
void midi_init();
void midi_task();
void mux_init();
void trigger_envelope(uint8_t voice);

// constants
const uint8_t mux_cs[NUM_MUX] = {PIN_CS_MUX_0, PIN_CS_MUX_1};
const uint8_t mux_bit[3] = {PIN_CH_MUX_A, PIN_CH_MUX_B, PIN_CH_MUX_C};

// system variables
uint8_t cv_cursor = 0;
uint16_t cv[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t env_lvl[NUM_VOICE] = {0, 0};
uint8_t env_phase[NUM_VOICE] = {0, 0};
uint32_t env_tick[NUM_VOICE] = {0, 0};
uint32_t led_tick = 0;
uint32_t lfo_del[NUM_VOICE] = {0, 0};
uint32_t lfo_delay_tick = 0;
uint16_t lfo_lvl = 0;
uint32_t lfo_tick = 0;
uint16_t lfo_val = 0;

// patch variables
uint16_t env_a = 200; // time in ms
uint16_t env_d = 200; // time in ms
uint16_t env_s = 256; // level between 0 and RES
uint16_t env_r = 1000; // time in ms
float lfo_freq = 5; // hz
uint8_t lfo_wav = SAW;
uint32_t lfo_delay = 0;

//
int main() {
    stdio_init_all();
    dac_init();
    led_init();
    midi_init();
    mux_init();

    trigger_envelope(0);

    while (1) {
        midi_task();
        env_task();
        lfo_task();
        cv_task();
        dac_task();
        led_task();
    }
}

void cv_task() {
    // move to the next control voltage
    cv_cursor = cv_cursor == 15 ? 0 : cv_cursor + 1;

    // get the target mux device and device channel
    uint8_t device = cv_cursor / 8;
    uint8_t channel = cv_cursor % 8;

    // deselect all mux devices
    for (uint8_t i = 0; i < NUM_MUX; i++) gpio_put(mux_cs[i], 0);

    // set the ic channel select bits
    for (uint8_t i = 0; i < 3; i++) gpio_put(mux_bit[i], (channel >> i) & 1);

    // select the target mux device
    gpio_put(mux_cs[device], 1);
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

// test function
void dac_task() {
    int f = 4096 / RES;
    dac_set(env_lvl[0] * f);
}

void env_task() {
    uint32_t tick = time_us_32();

    for (uint8_t i = 0; i < NUM_VOICE; i++) {
        uint16_t phase = env_phase[i];

        if (phase == SUSTAIN) {
            env_lvl[i] = env_s;
        } else {
            // step in microseconds
            uint32_t step = 0;

            if (phase == ATTACK) {
                step = (env_a * 1000) / RES;
            } else if (phase == DECAY) {
                step = (env_d * 1000) / (RES - env_s);
            } else if (phase == RELEASE) {
                step = (env_r * 1000) / env_s;
            }

            // initialize the first envelope tick
            if (env_tick[i] == 0) {
                env_tick[i] = tick - step;
            }

            // crude but probably good enough
            if (tick - env_tick[i] >= step) {
                env_tick[i] += step;

                if (phase == ATTACK) {
                    env_lvl[i] += 1;

                    if (env_lvl[i] == RES) {
                        env_phase[i] = DECAY;
                    }
                } else if (phase == DECAY) {
                    env_lvl[i] -= 1;

                    if (env_lvl[i] == env_s) {
                        env_phase[i] = SUSTAIN;
                    }
                } else if (phase == RELEASE) {
                    env_lvl[i] -= 1;

                    if (env_lvl[i] == 0) {
                        env_phase[i] = IDLE;
                    }
                }
            }
        }
    }
}

// hz to microseconds
uint32_t hz_to_us(float hz) {
    return 1000000 / hz;
}

void init_pin(uint8_t pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

void led_init() {
    init_pin(LED_PIN);
    led_tick = time_us_32();
}

void led_task() {
    gpio_put(LED_PIN, time_us_32() - led_tick < BLINK_TIME ? 1 : 0);
}

void lfo_task() {
    uint32_t tick = time_us_32();
    uint32_t step = hz_to_us(lfo_freq) / RES;
    uint16_t HRES = RES / 2;

    // initialize the first lfo tick
    if (lfo_tick == 0) lfo_tick = tick - step;

    // crude but probably good enough
    if (tick - lfo_tick >= step) {
        lfo_tick += step;
        lfo_val += 1;

        if (lfo_wav == SAW) {
            // increase lfo level by one, uint8_t will overflow to 0 by itself
            lfo_lvl += 1;
        } else if (lfo_wav == TRI) {
            if (lfo_val < HRES) {
                lfo_lvl = lfo_val * 2;
            } else {
                lfo_lvl = HRES - ((lfo_val - HRES) * 2);
            }
        } else if (lfo_wav == SIN) {
            // map lfo value to a range of 0 to 360
            uint16_t x = lfo_val * (360.0 / RES);
            // lfo level will oscillate around HRES (between 0 and RES)
            lfo_lvl = HRES + ((sin(x * (M_PI / 180))) * HRES);
        } else if (lfo_wav == SQR && (lfo_val == HRES || lfo_val == RES)) {
            // flip the lfo level twice every period
            lfo_lvl = lfo_lvl == RES ? 0 : RES;
        } else if (lfo_wav == RND && lfo_val == RES) {
            // choose a random lfo value every period
            lfo_lvl = rand() % RES;
        }
    }
}

void midi_init() {

}

void midi_task() {

}

void mux_init() {
    // initialize chip select pins
    for (uint8_t i = 0; i < NUM_MUX; i++) init_pin(mux_cs[i]);

    // initialize channel select pins
    for (uint8_t i = 0; i < 3; i++) init_pin(mux_bit[i]);
}

void trigger_envelope(uint8_t voice) {
    env_phase[voice] = ATTACK;
}
