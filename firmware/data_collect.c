#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "uart.h"
#include "servo.h"
#include "i2c.h"
#include "vl53l0x.h"

// --- Sensor Pins ---
#define TRIG_BALL_PIN   PD7
#define ECHO_BALL_PIN   PD6
#define XSHUT_BALL_PIN  PC2
#define VL_BALL_ADDR    0x30

#define TRIG_SP_PIN     PC4
#define ECHO_SP_PIN     PC5
#define XSHUT_SP_PIN    PC3
#define VL_SP_ADDR      0x29

#define TOTAL_TRACK_LENGTH_MM    350
#define SETPOINT_TRACK_LENGTH_MM 280

// Sweep range
#define MIN_ANGLE 30
#define MAX_ANGLE 120
#define STEP_ANGLE 2
#define STEP_DELAY_MS 150

void hcsr04_init(void) {
    DDRD  |=  (1 << TRIG_BALL_PIN);
    DDRD  &= ~(1 << ECHO_BALL_PIN);
    PORTD &= ~(1 << TRIG_BALL_PIN);

    DDRC  |=  (1 << TRIG_SP_PIN);
    DDRC  &= ~(1 << ECHO_SP_PIN);
    PORTC &= ~(1 << TRIG_SP_PIN);
}

uint16_t hcsr04_read_distance(volatile uint8_t *trig_port, uint8_t trig_pin,
                              volatile uint8_t *echo_pind, uint8_t echo_pin) {
    uint16_t start, now, elapsed;

    *trig_port &= ~(1 << trig_pin);
    _delay_us(2);
    *trig_port |= (1 << trig_pin);
    _delay_us(10);
    *trig_port &= ~(1 << trig_pin);

    start = TCNT1;
    while (!(*echo_pind & (1 << echo_pin))) {
        now = TCNT1;
        elapsed = (now >= start) ? (now - start) : ((20000 - start) + now);
        if (elapsed > 5000) return 0xFFFF;
    }

    start = TCNT1;
    while (*echo_pind & (1 << echo_pin)) {
        now = TCNT1;
        elapsed = (now >= start) ? (now - start) : ((20000 - start) + now);
        if (elapsed > 25000) return 0xFFFF;
    }

    now = TCNT1;
    uint16_t duration_us = (now >= start) ? (now - start) : ((20000 - start) + now);
    return (uint16_t)(((uint32_t)duration_us * 1715UL) / 10000UL);
}

uint16_t hcsr04_read_ball_mm(void) {
    return hcsr04_read_distance(&PORTD, TRIG_BALL_PIN, &PIND, ECHO_BALL_PIN);
}

uint16_t hcsr04_read_setpoint_mm(void) {
    return hcsr04_read_distance(&PORTC, TRIG_SP_PIN, &PINC, ECHO_SP_PIN);
}

void vl53l0x_dual_init(uint8_t *ball_ok, uint8_t *sp_ok) {
    *ball_ok = 0;
    *sp_ok = 0;

    DDRC |= (1 << XSHUT_BALL_PIN) | (1 << XSHUT_SP_PIN);
    PORTC &= ~((1 << XSHUT_BALL_PIN) | (1 << XSHUT_SP_PIN));
    _delay_ms(30);

    PORTC |= (1 << XSHUT_BALL_PIN);
    _delay_ms(30);

    uint8_t id1 = 0;
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id1) == 1) {
        if (vl53l0x_set_address(VL53L0X_DEFAULT_ADDR, VL_BALL_ADDR)) {
            *ball_ok = 1;
        }
    }

    PORTC |= (1 << XSHUT_SP_PIN);
    _delay_ms(30);

    uint8_t id2 = 0;
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id2) == 1) {
        *sp_ok = 1;
    }
}

int main(void) {
    // Disable JTAG
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    pwm_init();
    uart_init();
    hcsr04_init();
    i2c_init();

    uint8_t vl_ball_ok = 0, vl_sp_ok = 0;
    vl53l0x_dual_init(&vl_ball_ok, &vl_sp_ok);

    uart_println("\r\n==================================================================");
    uart_println(">>> DATA COLLECTION FIRMWARE: Full Servo Angle & Sensor Sweep <<<");
    uart_println("==================================================================");
    uart_println("HEADER,Angle_deg,US1_mm,VL1_raw,VL1_pos,Diff1,US2_mm,VL2_raw,VL2_pos,Diff2");

    char buf[180];
    int8_t dir = 1;
    uint8_t angle = MIN_ANGLE;

    while (1) {
        servo_set_angle(angle);
        _delay_ms(STEP_DELAY_MS);

        uint16_t us_ball = hcsr04_read_ball_mm();
        if (us_ball != 0xFFFF && us_ball > 1000) us_ball = 0xFFFF;

        uint16_t vl_ball_raw = vl_ball_ok ? vl53l0x_read_distance_addr(VL_BALL_ADDR) : 0xFFFF;
        if (vl_ball_raw != 0xFFFF && vl_ball_raw > 400) vl_ball_raw = 40;

        uint16_t us_sp = hcsr04_read_setpoint_mm();
        if (us_sp != 0xFFFF && us_sp > 1000) us_sp = 0xFFFF;

        uint16_t vl_sp_raw = vl_sp_ok ? vl53l0x_read_distance_addr(VL_SP_ADDR) : 0xFFFF;
        if (vl_sp_raw != 0xFFFF && vl_sp_raw > 400) vl_sp_raw = 40;

        int16_t vl1_pos = (vl_ball_raw != 0xFFFF) ? (TOTAL_TRACK_LENGTH_MM - (int16_t)vl_ball_raw) : -1;
        int16_t vl2_pos = (vl_sp_raw != 0xFFFF) ? (SETPOINT_TRACK_LENGTH_MM - (int16_t)vl_sp_raw) : -1;

        int16_t diff1 = (us_ball != 0xFFFF && vl1_pos != -1) ? ((int16_t)us_ball - vl1_pos) : 0;
        int16_t diff2 = (us_sp != 0xFFFF && vl2_pos != -1) ? ((int16_t)us_sp - vl2_pos) : 0;

        // Structured CSV line output
        sprintf(buf, "CSV,%u,%u,%u,%d,%d,%u,%u,%d,%d",
                angle,
                (us_ball == 0xFFFF) ? 0 : us_ball,
                (vl_ball_raw == 0xFFFF) ? 0 : vl_ball_raw,
                vl1_pos,
                diff1,
                (us_sp == 0xFFFF) ? 0 : us_sp,
                (vl_sp_raw == 0xFFFF) ? 0 : vl_sp_raw,
                vl2_pos,
                diff2);
        uart_println(buf);

        // Advance angle
        if (dir > 0) {
            if (angle + STEP_ANGLE >= MAX_ANGLE) {
                angle = MAX_ANGLE;
                dir = -1;
            } else {
                angle += STEP_ANGLE;
            }
        } else {
            if (angle <= MIN_ANGLE + STEP_ANGLE) {
                angle = MIN_ANGLE;
                dir = 1;
            } else {
                angle -= STEP_ANGLE;
            }
        }
    }

    return 0;
}
