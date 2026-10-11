#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include "servo.h"

void servo_init(void)
{
    // PD5 = OC1A
    DDRD |= (1 << PD5);

    // Fast PWM Mode 14
    // TOP = ICR1
    TCCR1A = (1 << COM1A1) | (1 << WGM11);

    TCCR1B = (1 << WGM13) |
             (1 << WGM12) |
             (1 << CS11);

    // 8 MHz / 8 = 1 MHz
    // 1 tick = 1 us
    // 20 ms period
    ICR1 = 19999;

    // Start around 40 degrees
    OCR1A = 400 + ((uint32_t)40 * 2000) / 180;
}

void pwm_init(void)
{
    servo_init();
}

void servo_set_angle(uint16_t angle)
{
    if (angle > 180)
        angle = 180;

    uint16_t pulse =
        400 + ((uint32_t)angle * 2000) / 180;

    OCR1A = pulse;
}