#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

// Default VL53L0X I2C address (7-bit)
#define VL53L0X_DEFAULT_ADDR 0x29

/**
 * @brief Sets a new I2C 7-bit address for a VL53L0X sensor currently at current_addr.
 * @param current_addr Current 7-bit I2C address (usually 0x29).
 * @param new_addr New 7-bit I2C address to assign.
 * @return 1 on success, 0 on failure.
 */
uint8_t vl53l0x_set_address(uint8_t current_addr, uint8_t new_addr);

/**
 * @brief Initializes VL53L0X distance sensor at a specific 7-bit I2C address.
 * @param addr 7-bit I2C address of target VL53L0X sensor.
 * @param id_out Pointer to receive Model ID byte (reg 0xC0).
 * @return 1 on success, non-1 on failure status code.
 */
uint8_t vl53l0x_init_addr(uint8_t addr, uint8_t *id_out);

/**
 * @brief Reads measured distance in millimeters from VL53L0X at a specific 7-bit address.
 * @param addr 7-bit I2C address of target VL53L0X sensor.
 * @return Distance in mm, or 0xFFFF on error / timeout.
 */
uint16_t vl53l0x_read_distance_addr(uint8_t addr);

// --- Backward compatibility single-sensor wrappers (using default 0x29) ---
uint8_t vl53l0x_init(void);
uint8_t vl53l0x_init_verbose(uint8_t *id_out);
uint16_t read_distance_mm(void);
uint16_t vl53l0x_read_distance_mm(void);
uint16_t vl53l0x_read_distance_verbose(uint8_t *status_out);

#endif // VL53L0X_H

