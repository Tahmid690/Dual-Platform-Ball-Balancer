#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"
#include "vl53l0x.h"

static uint8_t active_i2c_addr = 0x29;
static uint8_t stop_variable = 0;

/* Register addresses (from Pololu VL53L0X library) */
#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define GLOBAL_CONFIG_REF_EN_START_SELECT            0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89


/* ═══════════════════════════════════════════════ */
/* I2C Read/Write Wrappers                         */
/* ═══════════════════════════════════════════════ */
static uint8_t vl_wr(uint8_t reg, uint8_t val) {
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1)) { i2c_stop(); return 0; }
    if (!i2c_write(reg))                 { i2c_stop(); return 0; }
    if (!i2c_write(val))                 { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint8_t vl_wr16(uint8_t reg, uint16_t val) {
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1)) { i2c_stop(); return 0; }
    if (!i2c_write(reg))                 { i2c_stop(); return 0; }
    if (!i2c_write((val >> 8) & 0xFF))   { i2c_stop(); return 0; }
    if (!i2c_write(val & 0xFF))          { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint8_t vl_rd(uint8_t reg, uint8_t *val) {
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1))          { i2c_stop(); return 0; }
    if (!i2c_write(reg))                        { i2c_stop(); return 0; }
    i2c_stop();
    _delay_us(10);
    if (!i2c_start())                           return 0;
    if (!i2c_write((active_i2c_addr << 1) | 1)) { i2c_stop(); return 0; }
    if (!i2c_read_nack(val))                    { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint8_t vl_rd16(uint8_t reg, uint16_t *val) {
    uint8_t msb = 0, lsb = 0;
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1))          { i2c_stop(); return 0; }
    if (!i2c_write(reg))                        { i2c_stop(); return 0; }
    i2c_stop();
    _delay_us(10);
    if (!i2c_start())                           return 0;
    if (!i2c_write((active_i2c_addr << 1) | 1)) { i2c_stop(); return 0; }
    if (!i2c_read_ack(&msb))                    { i2c_stop(); return 0; }
    if (!i2c_read_nack(&lsb))                   { i2c_stop(); return 0; }
    i2c_stop();
    *val = ((uint16_t)msb << 8) | lsb;
    return 1;
}

static uint8_t vl_rd_multi(uint8_t reg, uint8_t *dst, uint8_t count) {
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1))          { i2c_stop(); return 0; }
    if (!i2c_write(reg))                        { i2c_stop(); return 0; }
    i2c_stop();
    _delay_us(10);
    if (!i2c_start())                           return 0;
    if (!i2c_write((active_i2c_addr << 1) | 1)) { i2c_stop(); return 0; }
    for (uint8_t i = 0; i < count - 1; i++) {
        if (!i2c_read_ack(&dst[i]))             { i2c_stop(); return 0; }
    }
    if (!i2c_read_nack(&dst[count - 1]))        { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint8_t vl_wr_multi(uint8_t reg, const uint8_t *src, uint8_t count) {
    if (!i2c_start()) return 0;
    if (!i2c_write(active_i2c_addr << 1)) { i2c_stop(); return 0; }
    if (!i2c_write(reg))                 { i2c_stop(); return 0; }
    for (uint8_t i = 0; i < count; i++) {
        if (!i2c_write(src[i]))          { i2c_stop(); return 0; }
    }
    i2c_stop();
    return 1;
}


/* ═══════════════════════════════════════════════ */
/* SPAD & Calibration Helpers                      */
/* ═══════════════════════════════════════════════ */
static uint8_t spad_get_info(uint8_t *cnt, uint8_t *is_aperture) {
    uint8_t tmp = 0;
    vl_wr(0x80, 0x01); vl_wr(0xFF, 0x01); vl_wr(0x00, 0x00);
    vl_wr(0xFF, 0x06);
    vl_rd(0x83, &tmp); vl_wr(0x83, tmp | 0x04);
    vl_wr(0xFF, 0x07); vl_wr(0x81, 0x01); vl_wr(0x80, 0x01);
    vl_wr(0x94, 0x6B); vl_wr(0x83, 0x00);

    uint16_t timeout = 3000;
    while (timeout--) {
        if (vl_rd(0x83, &tmp) && tmp) break;
        _delay_ms(1);
    }
    if (!tmp) return 0;

    vl_wr(0x83, 0x01);
    vl_rd(0x92, &tmp);
    *cnt        = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    vl_wr(0x81, 0x00); vl_wr(0xFF, 0x06);
    vl_rd(0x83, &tmp); vl_wr(0x83, tmp & ~0x04);
    vl_wr(0xFF, 0x01); vl_wr(0x00, 0x01);
    vl_wr(0xFF, 0x00); vl_wr(0x80, 0x00);
    return 1;
}

static void spad_configure(uint8_t cnt, uint8_t is_aperture) {
    uint8_t ref_spad_map[6];
    vl_rd_multi(DYNAMIC_SPAD_REF_EN_START_OFFSET, ref_spad_map, 6);

    vl_wr(0xFF, 0x01);
    vl_wr(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    vl_wr(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    vl_wr(0xFF, 0x00);
    vl_wr(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad = is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        uint8_t b = i / 8, bit = i % 8;
        if (i < first_spad || enabled == cnt) {
            ref_spad_map[b] &= ~(1 << bit);
        } else if ((ref_spad_map[b] >> bit) & 1) {
            enabled++;
        }
    }

    if (enabled == 0) {
        for (uint8_t i = 0; i < 6; i++) ref_spad_map[i] = 0;
        for (uint8_t i = first_spad; i < (uint8_t)(first_spad + cnt) && i < 48; i++) {
            ref_spad_map[i / 8] |= (1 << (i % 8));
        }
    }
    vl_wr_multi(DYNAMIC_SPAD_REF_EN_START_OFFSET, ref_spad_map, 6);
}

static uint8_t perform_single_ref_calibration(uint8_t vhv_init_byte) {
    vl_wr(SYSRANGE_START, 0x01 | vhv_init_byte);
    uint16_t timeout = 3000;
    uint8_t status = 0;
    while (timeout--) {
        if (vl_rd(RESULT_INTERRUPT_STATUS, &status) && (status & 0x07)) break;
        _delay_ms(1);
    }
    if (!(status & 0x07)) return 0;
    vl_wr(SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl_wr(SYSRANGE_START, 0x00);
    return 1;
}

/* ═══════════════════════════════════════════════ */
/* Public Sensor Interface                         */
/* ═══════════════════════════════════════════════ */
uint8_t vl53l0x_init_verbose(uint8_t *id_out) {
    _delay_ms(100); // Generous boot time

    /* Step 1: Read and verify model ID */
    uint8_t id = 0;
    for (uint8_t retry = 0; retry < 10; retry++) {
        if (vl_rd(0xC0, &id) && id != 0x00) break;
        _delay_ms(20);
    }
    if (id_out) *id_out = id;

    /* Verify I2C write works */
    if (!vl_wr(0x80, 0x01)) return 2;
    vl_wr(0x80, 0x00);

    /* Step 2: Set I/O voltage to 2.8V mode (CRITICAL for 3.3V/5V operation) */
    uint8_t vhv_cfg = 0;
    vl_rd(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &vhv_cfg);
    vl_wr(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, vhv_cfg | 0x01);

    /* Step 3: Set I2C standard mode */
    vl_wr(0x88, 0x00);

    /* Step 4: Read stop_variable */
    vl_wr(0x80, 0x01); vl_wr(0xFF, 0x01); vl_wr(0x00, 0x00);
    vl_rd(0x91, &stop_variable);
    vl_wr(0x00, 0x01); vl_wr(0xFF, 0x00); vl_wr(0x80, 0x00);

    /* Step 5: Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    uint8_t msrc_cfg = 0;
    vl_rd(MSRC_CONFIG_CONTROL, &msrc_cfg);
    vl_wr(MSRC_CONFIG_CONTROL, msrc_cfg | 0x12);

    /* Step 6: Set final range signal rate limit to 0.25 MCPS */
    vl_wr16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32); // 0.25 * 128 = 32

    /* Step 7: SPAD Management */
    uint8_t spad_count = 0, spad_aperture = 0;
    if (!spad_get_info(&spad_count, &spad_aperture)) return 4;
    spad_configure(spad_count, spad_aperture);

    /* Step 8: ST "Default Tuning Settings" (from Pololu library, section 1) */
    vl_wr(0xFF, 0x01); vl_wr(0x00, 0x00);
    vl_wr(0xFF, 0x00); vl_wr(0x09, 0x00);
    vl_wr(0x10, 0x00); vl_wr(0x11, 0x00);
    vl_wr(0x24, 0x01); vl_wr(0x25, 0xFF);
    vl_wr(0x75, 0x00);

    vl_wr(0xFF, 0x01); vl_wr(0x4E, 0x2C);
    vl_wr(0x48, 0x00); vl_wr(0x30, 0x20);

    vl_wr(0xFF, 0x00);
    vl_wr(0x30, 0x09); vl_wr(0x54, 0x00);
    vl_wr(0x31, 0x04); vl_wr(0x32, 0x03);
    vl_wr(0x40, 0x83); vl_wr(0x46, 0x25);
    vl_wr(0x60, 0x00); vl_wr(0x27, 0x00);
    vl_wr(0x50, 0x06); vl_wr(0x51, 0x00);
    vl_wr(0x52, 0x96); vl_wr(0x56, 0x08);
    vl_wr(0x57, 0x30); vl_wr(0x61, 0x00);
    vl_wr(0x62, 0x00); vl_wr(0x64, 0x00);
    vl_wr(0x65, 0x00); vl_wr(0x66, 0xA0);

    vl_wr(0xFF, 0x01);
    vl_wr(0x22, 0x32); vl_wr(0x47, 0x14);
    vl_wr(0x49, 0xFF); vl_wr(0x4A, 0x00);

    vl_wr(0xFF, 0x00);
    vl_wr(0x7A, 0x0A); vl_wr(0x7B, 0x00);
    vl_wr(0x78, 0x21);

    vl_wr(0xFF, 0x01);
    vl_wr(0x23, 0x34); vl_wr(0x42, 0x00);
    vl_wr(0x44, 0xFF); vl_wr(0x45, 0x26);
    vl_wr(0x46, 0x05); vl_wr(0x40, 0x40);
    vl_wr(0x0E, 0x06); vl_wr(0x20, 0x1A);
    vl_wr(0x43, 0x40);

    vl_wr(0xFF, 0x00);
    vl_wr(0x34, 0x03); vl_wr(0x35, 0x44);

    vl_wr(0xFF, 0x01);
    vl_wr(0x31, 0x04); vl_wr(0x4B, 0x09);
    vl_wr(0x4C, 0x05); vl_wr(0x4D, 0x04);

    vl_wr(0xFF, 0x00);
    vl_wr(0x44, 0x00); vl_wr(0x45, 0x20);
    vl_wr(0x47, 0x08); vl_wr(0x48, 0x28);
    vl_wr(0x67, 0x00); vl_wr(0x70, 0x04);
    vl_wr(0x71, 0x01); vl_wr(0x72, 0xFE);
    vl_wr(0x76, 0x00); vl_wr(0x77, 0x00);

    vl_wr(0xFF, 0x01);
    vl_wr(0x0D, 0x01);

    vl_wr(0xFF, 0x00);
    vl_wr(0x80, 0x01); vl_wr(0x01, 0xF8);

    vl_wr(0xFF, 0x01);
    vl_wr(0x8E, 0x01); vl_wr(0x00, 0x01);

    vl_wr(0xFF, 0x00);
    vl_wr(0x80, 0x00);

    /* Step 9: GPIO Interrupt Configuration */
    vl_wr(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_hv_mux = 0;
    vl_rd(0x84, &gpio_hv_mux);
    vl_wr(0x84, gpio_hv_mux & ~0x10);  // active low
    vl_wr(SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* Step 10: Set measurement timing budget to default 33ms */
    /* The default sequence config enables TCC, DSS, MSRC, Pre-Range, Final Range */
    vl_wr(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* Step 11: Set MSRC timeout, pre-range timeout, final-range timeout */
    /* Use default timing budget values (33ms total measurement time) */
    /* These are the Pololu defaults which work on virtually all VL53L0X */
    vl_wr16(0x51, 0x0099); // MSRC target timeout
    vl_wr16(0x46, 0x0600); // pre-range timeout  
    vl_wr(0x44, 0x00); vl_wr(0x45, 0x20); // final range signal rate limit

    /* Step 12: VHV + Phase Calibration */
    vl_wr(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!perform_single_ref_calibration(0x40)) return 5; // VHV cal

    vl_wr(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!perform_single_ref_calibration(0x00)) return 5; // Phase cal

    /* Step 13: Restore full sequence config */
    vl_wr(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return 1; // Success!
}

uint8_t vl53l0x_init(void) {
    return (vl53l0x_init_verbose(0) == 1);
}

/* ═══════════════════════════════════════════════ */
/* Distance Reading                                */
/* ═══════════════════════════════════════════════ */
uint16_t read_distance_mm(void) {
    /* Pre-ranging sequence */
    vl_wr(0x80, 0x01); vl_wr(0xFF, 0x01); vl_wr(0x00, 0x00);
    vl_wr(0x91, stop_variable);
    vl_wr(0x00, 0x01); vl_wr(0xFF, 0x00); vl_wr(0x80, 0x00);

    /* Start single-shot ranging */
    vl_wr(SYSRANGE_START, 0x01);

    /* Wait for measurement to start */
    uint16_t timeout = 500;
    uint8_t start_val = 1;
    while (timeout--) {
        if (vl_rd(SYSRANGE_START, &start_val) && !(start_val & 0x01)) break;
        _delay_ms(1);
    }
    if (start_val & 0x01) return 0xFFFF;

    /* Wait for measurement complete interrupt */
    timeout = 500;
    uint8_t int_status = 0;
    while (timeout--) {
        if (vl_rd(RESULT_INTERRUPT_STATUS, &int_status) && (int_status & 0x07)) break;
        _delay_ms(1);
    }
    if (!(int_status & 0x07)) return 0xFFFF;

    /* Read distance from result registers */
    uint16_t dist = 0;
    vl_rd16(RESULT_RANGE_STATUS + 10, &dist);

    /* Clear interrupt */
    vl_wr(SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* Filter out "no target" sentinel */
    if (dist == 8191 || dist == 0) return 0xFFFF;

    return dist;
}

uint16_t vl53l0x_read_distance_verbose(uint8_t *status_out) {
    vl_wr(0x80, 0x01); vl_wr(0xFF, 0x01); vl_wr(0x00, 0x00);
    vl_wr(0x91, stop_variable);
    vl_wr(0x00, 0x01); vl_wr(0xFF, 0x00); vl_wr(0x80, 0x00);
    vl_wr(SYSRANGE_START, 0x01);

    uint16_t timeout = 500;
    uint8_t start_val = 1;
    while (timeout--) {
        if (vl_rd(SYSRANGE_START, &start_val) && !(start_val & 0x01)) break;
        _delay_ms(1);
    }
    if (start_val & 0x01) { if (status_out) *status_out = 0xFF; return 0xFFFF; }

    timeout = 500;
    uint8_t int_status = 0;
    while (timeout--) {
        if (vl_rd(RESULT_INTERRUPT_STATUS, &int_status) && (int_status & 0x07)) break;
        _delay_ms(1);
    }
    if (!(int_status & 0x07)) { if (status_out) *status_out = 0xFE; return 0xFFFF; }

    uint8_t range_status = 0;
    vl_rd(RESULT_RANGE_STATUS, &range_status);
    if (status_out) *status_out = (range_status >> 3) & 0x0F;

    uint16_t dist = 0;
    vl_rd16(RESULT_RANGE_STATUS + 10, &dist);
    vl_wr(SYSTEM_INTERRUPT_CLEAR, 0x01);
    return dist;
}

uint16_t vl53l0x_read_distance_mm(void) {
    return read_distance_mm();
}

uint8_t vl53l0x_set_address(uint8_t current_addr, uint8_t new_addr) {
    active_i2c_addr = current_addr;
    uint8_t res = vl_wr(0x8A, new_addr & 0x7F); // 0x8A = I2C_SLAVE_DEVICE_ADDRESS
    if (res) {
        active_i2c_addr = new_addr;
    }
    return res;
}

uint8_t vl53l0x_init_addr(uint8_t addr, uint8_t *id_out) {
    active_i2c_addr = addr;
    return vl53l0x_init_verbose(id_out);
}

uint16_t vl53l0x_read_distance_addr(uint8_t addr) {
    active_i2c_addr = addr;
    return read_distance_mm();
}

