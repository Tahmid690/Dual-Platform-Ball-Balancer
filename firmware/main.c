#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#include "uart.h"
#include "servo.h"
#include "i2c.h"
#include "vl53l0x.h"

/* ═════════════════════════════════════════════════════════════════════════════════ */
/* Hardware & Pin Definitions                                                       */
/* ═════════════════════════════════════════════════════════════════════════════════ */

// --- Quad VL53L0X ToF Sensors (Shared I2C Bus: SDA=PC1, SCL=PC0) ---
// Pin mapping per request:
// S1 (PC2), S2 (PC3) -> Setpoint Sensors
// S3 (PC4), S4 (PC5) -> Ball Sensors

#define XSHUT_SP1_PIN    PC2  // Setpoint Sensor 1 (S1, Right End)
#define XSHUT_SP2_PIN    PC3  // Setpoint Sensor 2 (S2, Left End)
#define XSHUT_BALL1_PIN  PC4  // Ball Sensor 1 (S3, Right End)
#define XSHUT_BALL2_PIN  PC5  // Ball Sensor 2 (S4, Left End)

// Dynamic 7-bit I2C Addresses assigned at runtime
#define VL_SP1_ADDR      0x30  // Setpoint 1 (S1)
#define VL_SP2_ADDR      0x31  // Setpoint 2 (S2)
#define VL_BALL1_ADDR    0x32  // Ball 1 (S3)
#define VL_BALL2_ADDR    0x29  // Ball 2 (S4 - Retains default address 0x29)

// Track & Physical Boundaries (in mm)
#define TOTAL_TRACK_LENGTH_MM    360
#define SETPOINT_TRACK_LENGTH_MM 300  // Setpoint sensor track length
#define DEFAULT_SETPOINT_MM      100  // Center fallback setpoint
#define MIN_SETPOINT_MM          0
#define MAX_SETPOINT_MM          300

// PID Control Parameters
#define KP 15
#define KI 1
#define KD 25

#define BASE_ANGLE   70
#define DEADZONE_MM  20

// Kinematic Predictor Settings
#define MAX_PREDICTION_STEPS 15

// Global PID State
static int16_t prev_error = 0;
static int32_t integral = 0;

// Lightweight UART number formatters to prevent stack overflow
static void uart_print_uint(uint16_t val) {
    char buf[6];
    int i = 0;
    if (val == 0) {
        uart_tx('0');
        return;
    }
    while (val > 0 && i < 5) {
        buf[i++] = (char)('0' + (val % 10));
        val /= 10;
    }
    while (i > 0) {
        uart_tx(buf[--i]);
    }
}

static void uart_print_int16(int16_t val) {
    if (val < 0) {
        uart_tx('-');
        val = -val;
    }
    uart_print_uint((uint16_t)val);
}

static void uart_print_uint_pad(uint16_t val, uint8_t width) {
    if (val == 0xFFFF) {
        for (uint8_t i = 0; i < width - 3; i++) uart_tx(' ');
        uart_print("ERR");
        return;
    }
    uint16_t temp = val;
    uint8_t digits = 1;
    while (temp >= 10) { digits++; temp /= 10; }
    while (digits < width) { uart_tx(' '); digits++; }
    uart_print_uint(val);
}

static void uart_print_int_pad(int16_t val, uint8_t width) {
    int16_t temp = val;
    uint8_t digits = (temp < 0) ? 2 : 1;
    if (temp < 0) temp = -temp;
    while (temp >= 10) { digits++; temp /= 10; }
    while (digits < width) { uart_tx(' '); digits++; }
    uart_print_int16(val);
}

/* ═════════════════════════════════════════════════════════════════════════════════ */
/* Quad VL53L0X Sensor Initialization Routine                                       */
/* ═════════════════════════════════════════════════════════════════════════════════ */
void vl53l0x_quad_init(uint8_t *sp1_ok, uint8_t *sp2_ok, uint8_t *b1_ok, uint8_t *b2_ok) {
    *sp1_ok = 0;
    *sp2_ok = 0;
    *b1_ok = 0;
    *b2_ok = 0;

    // 1. Configure XSHUT pins as outputs on PORTC
    DDRC |= (1 << XSHUT_SP1_PIN)   | (1 << XSHUT_SP2_PIN) |
            (1 << XSHUT_BALL1_PIN) | (1 << XSHUT_BALL2_PIN);

    // 2. Step 1: Put ALL 4 sensors in hardware reset mode (XSHUT LOW) for 50ms
    PORTC &= ~((1 << XSHUT_SP1_PIN)   | (1 << XSHUT_SP2_PIN) |
               (1 << XSHUT_BALL1_PIN) | (1 << XSHUT_BALL2_PIN));
    _delay_ms(50);

    uint8_t id = 0;

    // 3. Step 2: Power up S1 (SP1 on PC2) -> change address 0x29 -> 0x30
    PORTC |= (1 << XSHUT_SP1_PIN);
    _delay_ms(30);
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id) == 1) {
        if (vl53l0x_set_address(VL53L0X_DEFAULT_ADDR, VL_SP1_ADDR)) {
            *sp1_ok = 1;
        }
    }

    // 4. Step 3: Power up S2 (SP2 on PC3) -> change address 0x29 -> 0x31
    PORTC |= (1 << XSHUT_SP2_PIN);
    _delay_ms(30);
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id) == 1) {
        if (vl53l0x_set_address(VL53L0X_DEFAULT_ADDR, VL_SP2_ADDR)) {
            *sp2_ok = 1;
        }
    }

    // 5. Step 4: Power up S3 (Ball 1 on PC4) -> change address 0x29 -> 0x32
    PORTC |= (1 << XSHUT_BALL1_PIN);
    _delay_ms(30);
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id) == 1) {
        if (vl53l0x_set_address(VL53L0X_DEFAULT_ADDR, VL_BALL1_ADDR)) {
            *b1_ok = 1;
        }
    }

    // 6. Step 5: Power up S4 (Ball 2 on PC5) -> keep default address 0x29
    PORTC |= (1 << XSHUT_BALL2_PIN);
    _delay_ms(30);
    if (vl53l0x_init_addr(VL53L0X_DEFAULT_ADDR, &id) == 1) {
        *b2_ok = 1;
    }
}

/* ═════════════════════════════════════════════════════════════════════════════════ */
/* Main Control Loop                                                                */
/* ═════════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    // CRITICAL: Disable JTAG to release PC2-PC5 for GPIO use.
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);

    pwm_init();    // Servo Timer1 (1 tick = 1 us, 20ms period)
    uart_init();   // UART @ 9600 baud
    i2c_init();    // Hardware TWI / I2C (PC0=SCL, PC1=SDA)

    servo_set_angle(BASE_ANGLE);

    uart_println("\r\n==================================================================");
    uart_println(">>> Quad VL53L0X ToF Ball Balancer (4x Laser Sensors) <<<");
    uart_println("==================================================================");
    uart_println("Track Length: 320 mm | Dynamic Setpoint from Dual VL53L0X Sensors");
    uart_println("Pins: I2C (PC0/PC1), XSHUT: SP1(PC2), SP2(PC3), Ball1(PC4), Ball2(PC5)");
    uart_println("------------------------------------------------------------------\r\n");

    uint8_t sp1_ok = 0, sp2_ok = 0, b1_ok = 0, b2_ok = 0;
    vl53l0x_quad_init(&sp1_ok, &sp2_ok, &b1_ok, &b2_ok);

    uart_print("Status -> SP1(0x30): "); uart_print(sp1_ok ? "OK" : "ERR");
    uart_print(" | SP2(0x31): "); uart_print(sp2_ok ? "OK" : "ERR");
    uart_print(" | Ball1(0x32): "); uart_print(b1_ok ? "OK" : "ERR");
    uart_print(" | Ball2(0x29): "); uart_print(b2_ok ? "OK" : "ERR");
    uart_println("\r\nStarting Quad-VL53L0X PID Control Loop...\r\n");

    // Dynamic setpoint & kinematic prediction state
    int16_t current_setpoint_mm = DEFAULT_SETPOINT_MM;
    int16_t pending_setpoint_mm = DEFAULT_SETPOINT_MM;
    uint8_t sp_change_count     = 0;
    int16_t last_valid_ball_mm  = DEFAULT_SETPOINT_MM;
    int16_t smoothed_velocity   = 0;
    uint8_t consecutive_failures = 0;

    while (1) {
        // 1. Read Setpoint Sensors (S1 = SP1 @ 0x30 Right End, S2 = SP2 @ 0x31 Left End)
        uint16_t sp1_dist = sp1_ok ? vl53l0x_read_distance_addr(VL_SP1_ADDR) : 0xFFFF;
        uint16_t sp2_dist = sp2_ok ? vl53l0x_read_distance_addr(VL_SP2_ADDR) : 0xFFFF;

        if (sp1_dist != 0xFFFF && sp1_dist > 400) sp1_dist = 0xFFFF;
        if (sp2_dist != 0xFFFF && sp2_dist > 400) sp2_dist = 0xFFFF;

        uint8_t sp1_valid = (sp1_dist != 0xFFFF && sp1_dist <= MAX_SETPOINT_MM);
        uint8_t sp2_valid = (sp2_dist != 0xFFFF && sp2_dist <= MAX_SETPOINT_MM);

        int16_t raw_sp_mm = -1;
        const char *sp_src = "DEFAULT";

        if (sp1_valid && sp2_valid) {
            if (sp1_dist <= sp2_dist) {
                raw_sp_mm = (int16_t)sp1_dist;
                sp_src = "SP1";
            } else {
                raw_sp_mm = SETPOINT_TRACK_LENGTH_MM - (int16_t)sp2_dist;
                sp_src = "SP2";
            }
        } else if (sp1_valid) {
            raw_sp_mm = (int16_t)sp1_dist;
            sp_src = "SP1";
        } else if (sp2_valid) {
            raw_sp_mm = SETPOINT_TRACK_LENGTH_MM -  (int16_t)sp2_dist;
            sp_src = "SP2";
        }

        // Sticky Consensus Filter: Only update setpoint after 3 consecutive confirming readings
        if (raw_sp_mm >= MIN_SETPOINT_MM && raw_sp_mm <= MAX_SETPOINT_MM) {
            int16_t candidate_sp = raw_sp_mm ;

            if (abs(candidate_sp - current_setpoint_mm) <= 3) {
                sp_change_count = 0;
            } else {
                if (abs(candidate_sp - pending_setpoint_mm) <= 5) {
                    sp_change_count++;
                    if (sp_change_count >= 3) {
                        current_setpoint_mm = candidate_sp;
                        sp_change_count = 0;
                    }
                } else {
                    pending_setpoint_mm = candidate_sp;
                    sp_change_count = 1;
                }
            }
        }

        // 2. Read Ball Sensors (S3 = Ball 1 @ 0x32 Right End, S4 = Ball 2 @ 0x29 Left End)
        uint16_t ball1_dist = b1_ok ? vl53l0x_read_distance_addr(VL_BALL1_ADDR) : 0xFFFF;
        uint16_t ball2_dist = b2_ok ? vl53l0x_read_distance_addr(VL_BALL2_ADDR) : 0xFFFF;

        if (ball1_dist != 0xFFFF && ball1_dist > 400) ball1_dist = 30;
        if (ball2_dist != 0xFFFF && ball2_dist > 400) ball2_dist = 31;

        uint8_t ball1_valid = (ball1_dist != 0xFFFF && ball1_dist <= 400);
        uint8_t ball2_valid = (ball2_dist != 0xFFFF && ball2_dist <= 400);

        int16_t ball1_pos_mm = ball1_valid ? ((int16_t)ball1_dist) : -1;
        int16_t ball2_pos_mm = ball2_valid ? (TOTAL_TRACK_LENGTH_MM - (int16_t)ball2_dist) : -1;

        int16_t raw_ball_mm = -1;
        const char *active_src = "NONE";

        if (ball1_valid && ball2_valid) {
            if (ball2_dist <= ball1_dist) {
                raw_ball_mm = ball2_pos_mm;
                active_src = "VL2";
            } else {
                raw_ball_mm = ball1_pos_mm;
                active_src = "VL1";
            }
        } else if (ball2_valid) {
            raw_ball_mm = ball2_pos_mm;
            active_src = "VL2";
        } else if (ball1_valid) {
            raw_ball_mm = ball1_pos_mm;
            active_src = "VL1";
        }

        // 3. Failsafe & Kinematic Prediction Logic for Ball Position
        int16_t ball_mm = 0;
        const char *state_str = "OK";

        if (raw_ball_mm < 0 || raw_ball_mm > 450) {
            consecutive_failures++;

            if (consecutive_failures <= MAX_PREDICTION_STEPS) {
                state_str = "PRD";
                ball_mm = last_valid_ball_mm + smoothed_velocity;

                if (ball_mm > TOTAL_TRACK_LENGTH_MM) ball_mm = TOTAL_TRACK_LENGTH_MM;
                if (ball_mm < 0) ball_mm = 0;

                smoothed_velocity = (smoothed_velocity * 8) / 10; // 80% momentum decay
                last_valid_ball_mm = ball_mm;
            } else {
                state_str = "ERR";
                smoothed_velocity = 0;
                integral = 0;
                ball_mm = current_setpoint_mm;
            }
        } else {
            state_str = active_src;
            int16_t instant_velocity = raw_ball_mm - last_valid_ball_mm;

            if (consecutive_failures == 0) {
                smoothed_velocity = (smoothed_velocity * 3 + instant_velocity) / 4;
            } else {
                smoothed_velocity = instant_velocity;
            }

            last_valid_ball_mm = raw_ball_mm;
            consecutive_failures = 0;
            ball_mm = raw_ball_mm;
        }

        // 4. Calculate PID Error relative to DYNAMIC SETPOINT
        int16_t error = current_setpoint_mm - ball_mm;

        // Apply Deadzone
        if (abs(error) < DEADZONE_MM) {
            error = 0;
        }

        // Proportional Term
        int32_t p_term = (int32_t)KP * error;

        // Integral Term with Anti-Windup Clamping
        integral += error;
        if (integral > 3000) integral = 3000;
        if (integral < -3000) integral = -3000;
        if (error == 0) integral = 0;

        int32_t i_term = (int32_t)(KI * integral);

        // Derivative Term
        int32_t d_term = (int32_t)KD * (error - prev_error);
        prev_error = error;

        // Calculate PID Output and Target Servo Angle
        int32_t pid_output = (p_term + i_term + d_term) / 100;
        int16_t target_angle = BASE_ANGLE + pid_output;

        // Clamp Target Servo Angle for mechanical safety
        if (target_angle > 100) target_angle = 100;
        if (target_angle < 0)   target_angle = 0;

        // Update Servo Position
        servo_set_angle((uint16_t)target_angle);

        // Send Telemetry over Serial Output (All 4 VL sensors displayed)
        uart_print("["); uart_print(state_str); uart_print("|"); uart_print(sp_src); uart_print("] ");
        uart_print("SP1:"); uart_print_uint_pad(sp1_dist == 0xFFFF ? 0xFFFF : (sp1_dist), 3);
        uart_print(" SP2:"); uart_print_uint_pad(sp2_dist == 0xFFFF ? 0xFFFF : SETPOINT_TRACK_LENGTH_MM - sp2_dist, 3);
        uart_print(" | Ball1:"); uart_print_uint_pad(ball1_dist == 0xFFFF ? 0xFFFF : (ball1_dist), 3);
        uart_print(" Ball2:"); uart_print_uint_pad(ball2_dist == 0xFFFF ? 0xFFFF : TOTAL_TRACK_LENGTH_MM - ball2_dist, 3);
        uart_print(" | Ball:"); uart_print_int_pad(ball_mm, 3); uart_print("mm");
        uart_print(" | Set:"); uart_print_int_pad(current_setpoint_mm, 3); uart_print("mm");
        uart_print(" | Err:"); uart_print_int_pad(error, 4);
        uart_print(" | Ang:"); uart_print_int_pad(target_angle, 2); uart_println("deg");

        _delay_ms(2); // 50 Hz loop execution rate
    }

    return 0;
}
