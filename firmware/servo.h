#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/**
 * @brief Initializes Timer1 Fast PWM Mode (50Hz) on PD5 (OC1A pin) for Servo control.
 */
void servo_init(void);
void pwm_init(void);

/**
 * @brief Sets servo motor angle (0 - 180 degrees).
 * @param angle Target angle in degrees.
 */
void servo_set_angle(uint16_t angle);

#endif // SERVO_H
