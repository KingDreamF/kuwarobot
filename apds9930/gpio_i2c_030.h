#ifndef __GPIO_I2C_030_H__
#define __GPIO_I2C_030_H__

#include "main.h"
#include "stm32f0xx_hal.h"
#include "stdint.h"

void     TWI_Initialize(void);
uint8_t TWI_START(void);
uint8_t TWI_START_SHT(void);
void TWI_STOP(void);
void TWI_NOP(void);
uint8_t  TWI_SendByte(uint8_t Data);
uint8_t  TWI_ReceiveByte(void);

#define TWI_ACK       0
#define TWI_READY     0
#define TWI_NACK      1
#define TWI_BUS_BUSY  2
#define TWI_BUS_ERROR 3

#endif