#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "bsp/board.h"
#include "master.pio.h"

#define BAUD_RATE 31250
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define GPIO_FUNC_UART 2
#define NUM_VOICES 2 //4
#define NUM_DCO 4 //8
#define MIDI_CHANNEL 1

// unsigned char midichar = 0; // midi character rx'd and/or to be tx'd
uint32_t led_blink_start = 0;

const uint8_t RESET_PINS[NUM_DCO] = {12, 13, 14, 15};
const uint8_t AMPLI_PINS[NUM_DCO] = {8, 9, 10, 11};
const uint8_t DCO_TO_PIO[NUM_DCO] = {0, 0, 0, 0};
const uint8_t DCO_TO_SM[NUM_DCO] = {0, 1, 2, 3};
const uint8_t VCA_PINS[NUM_VOICES] = {4, 5};
const uint8_t VCF_PINS[NUM_VOICES] = {6, 7};

const float BASE_NOTE = 440.0f;
const uint16_t DIV_COUNTER = 1250; // 100kHz base clock

// map a dco index to a pwm slice
uint8_t AMPLI_PWM_SLICES[NUM_DCO];

uint8_t VCA_PWM_SLICES[NUM_VOICES];
uint8_t VCF_PWM_SLICES[NUM_VOICES];

// keep track of when voices were triggered last
uint32_t VOICE_TIMES[NUM_VOICES] = {0, 0};

// contains the midi notes assigned to voices
uint8_t VOICE_NOTES[NUM_VOICES] = {0, 0};

uint16_t DCO_AMPLI[NUM_DCO] = {0, 0, 0, 0};

// contains gate on/off state of voices
uint8_t VOICE_GATES[NUM_VOICES] = {0, 0};

uint16_t VOICE_ENV[NUM_VOICES] = {0, 0};

// used with DCO_TO_PIO lookup
PIO pio[2] = {pio0, pio1};

uint16_t LFO = 0;
int lfo_inc = 1;
int clk = 0;

uint8_t next_voice = 0;
uint8_t midi_serial_status = 0;

// patch parameters
uint8_t range_1 = 0; // allowed: 0, 1, 2
uint8_t range_2 = 0; // allowed: 0, 1, 2
uint8_t tune_2 = 128; // +/- 1200 cent (12 semi-tones)
uint8_t fine_2 = 128; // +/- 50 cent (half a semi-tone)
uint8_t vcf_cutoff = 0;
uint8_t vcf_reso = 0;
uint8_t lfo_rate = 0;
uint8_t lfo_1 = 128;
uint8_t lfo_2 = 128;
uint8_t lfo_vcf = 0;
uint8_t env_1 = 0;
uint8_t env_2 = 0;
uint8_t env_vcf = 0;
uint8_t env_vca = 0;

void dac_init() {
    spi_init(spi_default, 20 * 1000 * 1000);
    spi_set_format(spi_default, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
}

void dac_set_value(uint16_t data) {
    uint16_t msg = 0b0011000000000000 | data;

    spi_write16_blocking(spi_default, msg, 1);
}

void led_blinking_task() {
    board_led_write(board_millis() - led_blink_start < 50);
}

void serial_midi_init() {
    // set UART speed.
	uart_init(UART_ID, BAUD_RATE);

	// set UART Tx and Rx pins
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

	// enable Tx and Rx fifos on UART
	uart_set_fifo_enabled(UART_ID, true);

	// disable cr/lf conversion on Tx
	uart_set_translate_crlf(UART_ID, false);
}

float get_freq_from_midi_note(uint8_t note) {
    return pow(2, (note - 69) / 12.0f) * BASE_NOTE;
}

void set_amplitude(uint8_t dco, float freq) {
    // this should result in a saw wave amplitude of 12 volts rail-tp-rail
    // measured at U104 pin 7 and pin 1
    uint16_t amplitude = (DIV_COUNTER * (freq * 0.00013f - 1 / (100 * freq)));

    DCO_AMPLI[dco] = amplitude;

    pwm_set_chan_level(
        AMPLI_PWM_SLICES[dco],
        pwm_gpio_to_channel(AMPLI_PINS[dco]),
        amplitude
    );
}

void set_frequency(uint8_t dco, float freq) {
    PIO p = pio[DCO_TO_PIO[dco]];
    uint sm = DCO_TO_SM[dco];

    uint32_t clk_div = clock_get_hz(clk_sys) / 2 / freq;

    if (freq == 0) clk_div = 0;

    pio_sm_put(p, sm, clk_div);
    pio_sm_exec(p, sm, pio_encode_pull(false, false));
    pio_sm_exec(p, sm, pio_encode_out(pio_y, 32));

    set_amplitude(dco, freq);
}

uint8_t get_free_voice() {
    uint32_t oldest_time = board_millis();
    uint8_t oldest_voice = 0;

    // Here we round robin the voices so each note will be played on the next
    // available (gate == 0) voice. If no voice is available, the voice that
    // was triggered the longest ago will be picked.
    for (int i = 0; i < NUM_VOICES; i++) {
        uint8_t n = (next_voice + i) % NUM_VOICES;

        if (VOICE_GATES[n] == 0) {
            next_voice = (n + 1) % NUM_VOICES;
            return n;
        }

        if (VOICE_TIMES[i] < oldest_time) {
            oldest_time = VOICE_TIMES[i];
            oldest_voice = i;
        }
    }

    next_voice = (oldest_voice + 1) % NUM_VOICES;
    return oldest_voice;
}

void voices_task() {
    for (int i = 0; i < NUM_VOICES; i++) {
        uint8_t note = VOICE_NOTES[i];
        uint16_t env = VOICE_ENV[i];

        // update dco frequencies
        uint8_t dco1 = i * 2;
        uint8_t dco2 = dco1 + 1;

        float freq = get_freq_from_midi_note(note);
        float tune_f = (tune_2 - 128) / 128;
        float fine_f = (fine_2 - 128) / (128 * 24);
        float lfo_1_f = LFO * (lfo_1 / 255) / 20000;
        float lfo_2_f = LFO * (lfo_2 / 255) / 20000;
        float env_1_f = env * (env_1 / 255);
        float env_2_f = env * (env_2 / 255);
        float f_1 = range_1 + lfo_1_f + env_1_f;
        float f_2 = range_2 + lfo_2_f + env_2_f + tune_f + fine_f;

        set_frequency(dco1, freq * pow(2, f_1));
        set_frequency(dco2, freq * pow(2, f_2));

        // todo: update vcf, vca
        uint16_t vcf = 65535;
        uint16_t vca = 65535;

        pwm_set_chan_level(
            VCA_PWM_SLICES[i], pwm_gpio_to_channel(VCA_PINS[i]), vca
        );

        pwm_set_chan_level(
            VCF_PWM_SLICES[i], pwm_gpio_to_channel(VCF_PINS[i]), vcf
        );
    }
}

void note_on(uint8_t note, uint8_t velocity) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (VOICE_NOTES[i] == note) return;
    }

    uint8_t voice_num = get_free_voice();

    VOICE_TIMES[voice_num] = board_millis();
    VOICE_NOTES[voice_num] = note;
    VOICE_GATES[voice_num] = 1;
}

void note_off(uint8_t note) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (VOICE_NOTES[i] == note) {
            VOICE_TIMES[i] = 0;
            VOICE_GATES[i] = 0;

            // test
            VOICE_NOTES[i] = 0;
        }
    }
}

void serial_midi_task() {
    if (!uart_is_readable(UART_ID)) return;

    uint8_t lsb = 0;
    uint8_t msb = 0;
    uint8_t data = uart_getc(UART_ID);

    led_blink_start = board_millis();

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

void lfo_task() {
    if (clk == 0) {
        if (LFO == 62500) {
            lfo_inc = -1;
        } else if (LFO == 0) {
            lfo_inc = 1;
        }

        LFO += lfo_inc;
    }
}

void clk_task() {
    clk += 1;

    if (clk == 1000) {
        clk = 0;
    }
}

int main() {
    stdio_init_all();
    board_init();
    // serial_midi_init();
    dac_init();

    // amplitude pwm init
    // for (int i = 0; i < NUM_DCO; i++) {
    //     gpio_set_function(AMPLI_PINS[i], GPIO_FUNC_PWM);
    //     AMPLI_PWM_SLICES[i] = pwm_gpio_to_slice_num(AMPLI_PINS[i]);
    //     pwm_set_wrap(AMPLI_PWM_SLICES[i], DIV_COUNTER);
    //     pwm_set_enabled(AMPLI_PWM_SLICES[i], true);
    // }

    // // vcf/vca pwm init
    // for (int i = 0; i < NUM_VOICES; i++) {
    //     // vca
    //     gpio_set_function(VCA_PINS[i], GPIO_FUNC_PWM);
    //     VCA_PWM_SLICES[i] = pwm_gpio_to_slice_num(VCA_PINS[i]);
    //     pwm_set_wrap(VCA_PWM_SLICES[i], DIV_COUNTER);
    //     pwm_set_enabled(VCA_PWM_SLICES[i], true);

    //     // vcf
    //     gpio_set_function(VCF_PINS[i], GPIO_FUNC_PWM);
    //     VCF_PWM_SLICES[i] = pwm_gpio_to_slice_num(VCF_PINS[i]);
    //     pwm_set_wrap(VCF_PWM_SLICES[i], DIV_COUNTER);
    //     pwm_set_enabled(VCF_PWM_SLICES[i], true);
    // }

    // // pio init
    // uint offset[2];
    // offset[0] = pio_add_program(pio[0], &frequency_program);
    // offset[1] = pio_add_program(pio[1], &frequency_program);

    // for (int i = 0; i < NUM_DCO; i++) {
    //     uint8_t p = DCO_TO_PIO[i];
    //     init_sm_pin(pio[p], DCO_TO_SM[i], offset[p], RESET_PINS[i]);
    //     pio_sm_set_enabled(pio[p], DCO_TO_SM[i], true);
    // }

    led_blink_start = board_millis();

    uint16_t value = 0;
    uint16_t meta = 0b0011000000000000;

    while (1) {
        if (value > 4095) {
            value = 0;
        }

        value += 16;

        uint16_t msg = meta | value;

        spi_write16_blocking(spi_default, &msg, 1);

        led_blinking_task();
    }

    // loop
    // while(1) {
    //     // lfo_task();
    //     // serial_midi_task();
    //     // voices_task();
    //     // led_blinking_task();
    //     // clk_task();
    // }
}