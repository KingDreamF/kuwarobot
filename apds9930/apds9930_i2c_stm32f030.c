//#include "stm32f0xx_hal_gpio.h"
//#include "stm32f0xx_hal_rcc.h"
//#include "stm32f0xx_hal_i2c.h"
#include "apds9930_i2c_stm32f030.h"
#include "gpio_i2c_030.h"

#define TIMEOUT 100000

#ifdef USE_GPIO_I2C

int apds9930_i2c_init(void)
{
	TWI_Initialize();
}

int32_t apds9930_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
	index = index | 0x80;

	if (TWI_START() != TWI_READY)
		return -1;

	TWI_SendByte(address << 1);
	TWI_SendByte(index);
	TWI_SendByte(data);
	TWI_STOP();

	return 1;
}
int32_t apds9930_read_byte(uint8_t address, uint8_t index, uint8_t  *pdata)
{
	index = index | 0x80;
	
	if (TWI_START() != TWI_READY)
		return -1;

	TWI_SendByte(address << 1);
	TWI_SendByte(index);

	if (TWI_START() != TWI_READY)
		return -1;

	TWI_SendByte( (address << 1) + 1);
	*pdata = TWI_ReceiveByte();
	TWI_STOP();
	return 1;
}


#else
/*  PF6---I2C2_SCL, PF7---I2C2_SDA  */
int apds9930_i2c_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	/* RCC Configuration */
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	/*I2C Peripheral clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* I2C GPIO clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	/* Reset I2Cx IP */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

	/* GPIO Configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_SetBits(GPIOF, GPIO_Pin_6 | GPIO_Pin_7);  // Pull up to free bus

													/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_5);
	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_5);

	I2C_DeInit(I2C2);

	/*!< I2C Struct Initialize */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_Timing = 0x20E32E44;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);
}

int32_t apds9930_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
	index = index | 0x80;
	int i = 0;
	int timeout = TIMEOUT;
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != SET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (0 == timeout)
	{
		return 0;
	}

	
	I2C_TransferHandling(I2C2, address<<1, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	timeout = TIMEOUT;
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) != RESET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (timeout == 0)
	{
		return 0;
	}
	I2C_SendData(I2C2, index);
	timeout = TIMEOUT;
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_TCR) != RESET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (timeout == 0)
	{
		return 0;
	}
	I2C_TransferHandling(I2C2, address<<1, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

		
	timeout = TIMEOUT;
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) == RESET)
	{
		//OSTimeDly(1);
		if (--timeout == 0)
		{
			return 0;
		}
	}
	I2C_SendData(I2C2, data);

	//I2C_TransferHandling(I2C2, address, 0, I2C_SoftEnd_Mode, I2C_Generate_Stop);
	timeout = TIMEOUT;
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET)
	{
		if ((timeout--) == 0)
		{
			return 0;
		}
	}
	return 1;

}
int32_t apds9930_read_byte(uint8_t address, uint8_t index, uint8_t  *pdata)
{
	index = index | 0x80;
	int timeout = TIMEOUT;
	/*uint8_t rt[6] = { 0, 0, 0, 0, 0, 0 };*/
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != SET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (0 == timeout)
	{
		return 0;
	} 

	timeout = TIMEOUT;
	I2C_TransferHandling(I2C2,address<<1, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_TXIS) != RESET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (timeout == 0)
	{		
		return 0;
	}
	I2C_SendData(I2C2, index);

	timeout = TIMEOUT;
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_TC) != RESET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (timeout == 0)
	{
		return 0;
	}
	I2C_TransferHandling(I2C2, address<<1, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	timeout = TIMEOUT;
	do
	{
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_RXNE) != RESET)
			break;
		//delay_ms(1);
		timeout--;
	} while (timeout);
	if (timeout == 0)
	{
		return 0;
	}
	*pdata = I2C_ReceiveData(I2C2);

	timeout = TIMEOUT;
	while (I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF) == RESET)
	{
		if ((timeout--) == 0)
		{
			return 0;
		}
	}
	I2C_ClearFlag(I2C2, I2C_ICR_STOPCF);
	//I2C_TransferHandling(I2C2, address, 0, I2C_SoftEnd_Mode, I2C_Generate_Stop);
//	return (rt[1] * 256 + rt[0]);
	return 1;
}

#endif