#ifndef __BUTTON__H
#define __BUTTON__H


#include "main.h"
#include "stm32f0xx_hal.h"


#define	DOWN 0X01
#define	UP 0X02

#define COUNTTIME 100

typedef struct 
{
	uint8_t flage;
	uint8_t short_press;
	uint8_t	long_press;
	uint32_t up_time;
	uint32_t down_time;
}Button;

extern Button button;

void buttonOperation();
#endif