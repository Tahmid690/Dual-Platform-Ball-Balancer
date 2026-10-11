#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"

void i2c_init(void) {
    // Set PC0 (SCL) and PC1 (SDA) as inputs with internal pull-ups enabled
    DDRC  &= ~((1 << PC0) | (1 << PC1));
    PORTC |=  (1 << PC0) | (1 << PC1);

    TWSR   = 0x00; // Prescaler = 1
    TWBR   = 72;   // ~40 kHz I2C clock (more tolerant of pull-up resistors & wiring noise)
    TWCR   = (1 << TWEN);
}

static uint8_t i2c_wait(void) {
    uint16_t timeout = 10000;
    while (!(TWCR & (1 << TWINT))) {
        if (!--timeout) return 0;
    }
    return 1;
}

uint8_t i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!i2c_wait()) return 0;
    uint8_t status = TWSR & 0xF8;
    return (status == 0x08 || status == 0x10);
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    _delay_us(100);
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!i2c_wait()) return 0;
    uint8_t status = TWSR & 0xF8;
    return (status == 0x18 || status == 0x28 || status == 0x40);
}

uint8_t i2c_read_ack(uint8_t *val) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    if (!i2c_wait()) return 0;
    *val = TWDR;
    return 1;
}

uint8_t i2c_read_nack(uint8_t *val) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (!i2c_wait()) return 0;
    *val = TWDR;
    return 1;
}
