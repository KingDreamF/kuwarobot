#ifndef APDS9930_I2C_STM32030_H
#define APDS9930_I2C_STM32030_H

#include "stdint.h"

#define USE_GPIO_I2C

int apds9930_i2c_init(void);
int32_t apds9930_write_byte(uint8_t address, uint8_t index, uint8_t data);
int32_t apds9930_read_byte(uint8_t address, uint8_t index, uint8_t  *pdata);



#endif