#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "synth.pio.h"

#define BAUD_RATE 31250
#define UART_ID uart0
#define PIN_UART_TX 0
#define PIN_UART_RX 1
#define PIN_CH_MUX_A 2
#define PIN_CH_MUX_B 3
#define PIN_CH_MUX_C 4
#define PIN_CS_MUX_0 5
#define PIN_CS_MUX_1 6
#define LED_PIN PICO_DEFAULT_LED_PIN
#define BLINK_TIME 100000

#define MIDI_CHANNEL 1
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
uint8_t get_free_voice();
float get_freq_from_midi_note(uint8_t note);
uint32_t hz_to_us(float hz);
void init_pin(uint8_t pin);
void led_init();
void led_task();
void lfo_task();
void midi_init();
void midi_task();
void mux_init();
void note_off(uint8_t note);
void note_on(uint8_t note, uint8_t velocity);
void pio_init();
void set_amplitude(uint8_t dco, float freq);
void set_frequency(uint8_t dco, float freq);
void trigger_envelope(uint8_t voice);
void voices_task();

// constants
const uint8_t reset_pins[NUM_DCO] = {12, 13, 14, 15};
const uint8_t dco_to_pio[NUM_DCO] = {0, 0, 0, 0};
const uint8_t dco_to_sm[NUM_DCO] = {0, 1, 2, 3};
const uint8_t mux_cs[NUM_MUX] = {PIN_CS_MUX_0, PIN_CS_MUX_1};
const uint8_t mux_bit[3] = {PIN_CH_MUX_A, PIN_CH_MUX_B, PIN_CH_MUX_C};
const PIO pio[2] = {pio0, pio1};
const float BASE_NOTE = 440.0f;

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
uint8_t midi_serial_status = 0;
uint8_t next_voice = 0;
uint32_t VOICE_TIMES[NUM_VOICE] = {0, 0};
uint8_t VOICE_NOTES[NUM_VOICE] = {0, 0};
uint8_t VOICE_GATES[NUM_VOICE] = {0, 0};

// patch variables
uint16_t env_a = 200; // time in ms
uint16_t env_d = 200; // time in ms
uint16_t env_s = 256; // level between 0 and RES
uint16_t env_r = 1000; // time in ms
float lfo_freq = 5; // hz
uint8_t lfo_wav = SAW;
uint32_t lfo_delay = 0;
uint8_t range_1 = 0; // allowed: 0, 1, 2
uint8_t range_2 = 0; // allowed: 0, 1, 2
uint8_t tune_2 = 128; // +/- 1200 cent (12 semi-tones)
uint8_t fine_2 = 128; // +/- 50 cent (half a semi-tone)
uint8_t vcf_cutoff = 0;
uint8_t vcf_reso = 0;
uint8_t lfo_1 = 128;
uint8_t lfo_2 = 128;
uint8_t lfo_vcf = 0;
uint8_t env_1 = 0;
uint8_t env_2 = 0;
uint8_t env_vcf = 0;
uint8_t env_vca = 0;

//
int main() {
    stdio_init_all();
    dac_init();
    led_init();
    midi_init();
    mux_init();
    pio_init();

    trigger_envelope(0);

    while (1) {
        midi_task();
        env_task();
        lfo_task();
        cv_task();
        dac_task();
        voices_task();
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

uint8_t get_free_voice() {
    uint32_t oldest_time = time_us_32();
    uint8_t oldest_voice = 0;

    // Here we round robin the voices so each note will be played on the next
    // available (gate == 0) voice. If no voice is available, the voice that
    // was triggered the longest ago will be picked.
    for (int i = 0; i < NUM_VOICE; i++) {
        uint8_t n = (next_voice + i) % NUM_VOICE;

        if (VOICE_GATES[n] == 0) {
            next_voice = (n + 1) % NUM_VOICE;
            return n;
        }

        if (VOICE_TIMES[i] < oldest_time) {
            oldest_time = VOICE_TIMES[i];
            oldest_voice = i;
        }
    }

    next_voice = (oldest_voice + 1) % NUM_VOICE;
    return oldest_voice;
}

float get_freq_from_midi_note(uint8_t note) {
    return pow(2, (note - 69) / 12.0f) * BASE_NOTE;
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
	uart_init(UART_ID, BAUD_RATE);

	gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
	gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

	uart_set_fifo_enabled(UART_ID, true);

	// disable cr/lf conversion on Tx
	uart_set_translate_crlf(UART_ID, false);
}

void midi_task() {
    if (!uart_is_readable(UART_ID)) return;

    uint8_t lsb = 0;
    uint8_t msb = 0;
    uint8_t data = uart_getc(UART_ID);

    led_tick = time_us_32();

    // status
    if (data >= 0xF0 && data <= 0xF7) {
        midi_serial_status = 0;
        return;
    }

    // realtime message
    if (data >= 0xF8 && data <= 0xFF) {
        return;
    }

    if (data >= 0x80 && data <= 0xEF) {
        midi_serial_status = data;
    }

    if (midi_serial_status >= 0x80 && midi_serial_status <= 0x9F ||
        midi_serial_status >= 0xB0 && midi_serial_status <= 0xBF || // cc mess
        midi_serial_status >= 0xE0 && midi_serial_status <= 0xEF) {

        lsb = uart_getc(UART_ID);
        msb = uart_getc(UART_ID);
    }

    if (midi_serial_status == (0x90 | (MIDI_CHANNEL - 1))) {
        if (msb > 0) {
            note_on(lsb, msb);
        } else {
            note_off(lsb);
        }
    }

    if (midi_serial_status == (0x80 | (MIDI_CHANNEL - 1))) {
        note_off(lsb);
    }
}

void mux_init() {
    // initialize chip select pins
    for (uint8_t i = 0; i < NUM_MUX; i++) init_pin(mux_cs[i]);

    // initialize channel select pins
    for (uint8_t i = 0; i < 3; i++) init_pin(mux_bit[i]);
}

void note_off(uint8_t note) {
    for (int i = 0; i < NUM_VOICE; i++) {
        if (VOICE_NOTES[i] == note) {
            VOICE_TIMES[i] = 0;
            VOICE_GATES[i] = 0;

            // test
            VOICE_NOTES[i] = 0;
        }
    }
}

void note_on(uint8_t note, uint8_t velocity) {
    for (int i = 0; i < NUM_VOICE; i++) {
        if (VOICE_NOTES[i] == note) return;
    }

    uint8_t voice_num = get_free_voice();

    VOICE_TIMES[voice_num] = time_us_32();
    VOICE_NOTES[voice_num] = note;
    VOICE_GATES[voice_num] = 1;
}

void pio_init() {
    // pio init
    uint offset[2];
    offset[0] = pio_add_program(pio[0], &frequency_program);
    offset[1] = pio_add_program(pio[1], &frequency_program);

    for (int i = 0; i < NUM_DCO; i++) {
        uint8_t p = dco_to_pio[i];
        init_sm_pin(pio[p], dco_to_sm[i], offset[p], reset_pins[i]);
        pio_sm_set_enabled(pio[p], dco_to_sm[i], true);
    }
}

void set_amplitude(uint8_t dco, float freq) {
    // this should result in a saw wave amplitude of 12 volts rail-tp-rail
    // measured at U104 pin 7 and pin 1

    // todo set level in cv array
}

void set_frequency(uint8_t dco, float freq) {
    PIO p = pio[dco_to_pio[dco]];
    uint sm = dco_to_sm[dco];

    uint32_t clk_div = clock_get_hz(clk_sys) / 2 / freq;

    if (freq == 0) clk_div = 0;

    pio_sm_put(p, sm, clk_div);
    pio_sm_exec(p, sm, pio_encode_pull(false, false));
    pio_sm_exec(p, sm, pio_encode_out(pio_y, 32));

    set_amplitude(dco, freq);
}

void trigger_envelope(uint8_t voice) {
    env_phase[voice] = ATTACK;
}

void voices_task() {
    for (int i = 0; i < NUM_VOICE; i++) {
        uint8_t note = VOICE_NOTES[i];
        uint16_t env = env_lvl[i];

        // update dco frequencies
        uint8_t dco1 = i * 2;
        uint8_t dco2 = dco1 + 1;

        float freq = get_freq_from_midi_note(note);
        // todo calculate boundaries
        float tune_f = (tune_2 - 128) / 128;
        float fine_f = (fine_2 - 128) / (128 * 24);
        float lfo_1_f = lfo_lvl * (lfo_1 / 255) / 20000;
        float lfo_2_f = lfo_lvl * (lfo_2 / 255) / 20000;
        float env_1_f = env * (env_1 / 255);
        float env_2_f = env * (env_2 / 255);
        float f_1 = range_1 + lfo_1_f + env_1_f;
        float f_2 = range_2 + lfo_2_f + env_2_f + tune_f + fine_f;

        set_frequency(dco1, freq * pow(2, f_1));
        set_frequency(dco2, freq * pow(2, f_2));
    }
}
