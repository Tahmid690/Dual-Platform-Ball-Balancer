#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c_init(void);
uint8_t i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(uint8_t *val);
uint8_t i2c_read_nack(uint8_t *val);

#endif // I2C_H
