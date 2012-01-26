#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#define BUTTON_INC      PB1
#define BUTTON_DEC      PB2

#define MAX_CURRENT_SET     0xFE

#define MAX_PWM_VALUE       0x78

#define MAX_CURRENT_DIFF    0x10
#define MIN_CURRENT         0x20

#define START_COUNTER_DELAY 0xFF

typedef enum BUTTON_STATE {
    BUTTON_INC_ON         =   1 << 0,
    BUTTON_DEC_ON         =   1 << 1
} BUTTON_STATE;

static uint8_t current = 0, voltage = 0;
static uint8_t voltage_buf[8] = {0,0,0,0,0,0,0,0}, current_buf[8] = {0,0,0,0,0,0,0,0};
static uint8_t current_set = 1;
static uint16_t start_counter = 0;

uint8_t EEMEM current_set_eeprom = 1;
uint8_t EEMEM pwm_value = 1;

#define USE_CURRENT_SENSE

void delay(uint8_t msec) {
    volatile uint16_t i;
    volatile uint8_t j;

    for (j = 0; j < msec; j++) {
        for (i = 0; i < ((F_CPU / 100000) / 2); i++) {
            asm volatile ( "nop;" );
        }
    }
}

void update_pwm() {
    uint8_t current_diff;

    if (current < current_set) {
        current_diff = current_set - current;

        if (current_diff > MAX_CURRENT_DIFF && current < MIN_CURRENT) {
            if (start_counter > START_COUNTER_DELAY) {
                OCR0A = 1;
            } else {
                if (OCR0A < MAX_PWM_VALUE) {
                    OCR0A++;
                }

                start_counter++;
            }
        } else {
            if (OCR0A < MAX_PWM_VALUE) {
                OCR0A++;
            }
        }
    }

    if (current > current_set) {
        current_diff = current - current_set;
        
//        OCR0A -= (current_diff >> 2) + 1;
        OCR0A--;
    }

}

ISR (ADC_vect) {
    uint16_t value = 0;
    uint8_t i;


    if (ADMUX & _BV(MUX0)) {
        for (i = sizeof(voltage_buf) - 1; i > 0; i--) {
            voltage_buf[i] = voltage_buf[i - 1];
            value += voltage_buf[i];
        }

        voltage_buf[0] = ADCH;
        value += ADCH;
        voltage = value / sizeof(voltage_buf);

        ADMUX &= ~_BV(MUX0);
    } else {
        for (i = sizeof(current_buf) - 1; i > 0; i--) {
            current_buf[i] = current_buf[i - 1];
            value += current_buf[i];
        }

        current_buf[0] = ADCH;
        value += ADCH;

        current = value / sizeof(current_buf);

        ADMUX |= _BV(MUX0);
        update_pwm();
    }

    ADCSRA |= _BV(ADSC);
}


int main(void) {
    uint8_t state = 0;

    PORTB = 0;
    DDRB = _BV(PB0);

	wdt_enable(WDTO_120MS);

    while (OSCCAL < 0x7D) {
        OSCCAL++;
        delay(10000);   
    }

    // Set PWM

    current_set = eeprom_read_byte((uint8_t*) &current_set_eeprom);

    OCR0A = eeprom_read_byte((uint8_t*) &pwm_value);
    //OCR0A = 1;

    TCCR0A = _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B = _BV(CS00);
    TCNT0 = 0;
    TIMSK0 = 0;

    // Set ADC

    ADMUX = _BV(MUX1) | _BV(REFS0) | _BV(ADLAR);
    ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    DIDR0 = _BV(ADC2D) | _BV(ADC3D);

#ifdef USE_CURRENT_SENSE
    ADCSRA |= _BV(ADSC);
#endif

    sei();

//    OSCCAL = 0x7F;

	while (1) {
       	wdt_reset();

        if (OCR0A > MAX_PWM_VALUE) {
            OCR0A = MAX_PWM_VALUE;
        }
            

        // Check button state ----------

        if ((PINB & _BV(BUTTON_INC)) && (state & BUTTON_INC_ON)) {
            state &= ~(BUTTON_INC_ON);

#ifndef USE_CURRENT_SENSE
            if (OCR0A < 0xFE) {
                OCR0A++;
                eeprom_write_byte((uint8_t*) &pwm_value, OCR0A);
            }
#endif

#ifdef USE_CURRENT_SENSE
            if (current_set < MAX_CURRENT_SET) {
                current_set++;
                eeprom_write_byte((uint8_t *) &current_set_eeprom, current_set);
                eeprom_write_byte((uint8_t*) &pwm_value, OCR0A);
            }
#endif
            // INREMENT
        } 

        if (!(PINB & _BV(BUTTON_INC)) && !(state & BUTTON_INC_ON)) {
            state |= BUTTON_INC_ON;
        }

        if ((PINB & _BV(BUTTON_DEC)) && (state & BUTTON_DEC_ON)) {
            state &= ~(BUTTON_DEC_ON);

#ifndef USE_CURRENT_SENSE
            if (OCR0A > 0) {
                OCR0A--;
                eeprom_write_byte((uint8_t*) &pwm_value, OCR0A);
            }
#endif

#ifdef USE_CURRENT_SENSE
            if (current_set > 1) {
                current_set--;
                eeprom_write_byte((uint8_t *) &current_set_eeprom, current_set);
                eeprom_write_byte((uint8_t*) &pwm_value, OCR0A);
            }
#endif
            // DECREMENT
        } 

        if (!(PINB & _BV(BUTTON_DEC)) && !(state & BUTTON_DEC_ON)) {
            state |= BUTTON_DEC_ON;
        }

        // -----------------------------

        delay(10000);
	}

	return 0;
}
