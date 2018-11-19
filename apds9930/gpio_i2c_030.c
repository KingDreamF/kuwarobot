#include "gpio_i2c_030.h"

#define PIN_SCL   GPIO_PIN_8  
#define PIN_SDA   GPIO_PIN_9  
#define PORT_I2C  GPIOB


static inline void TWI_SCL_0(void)        { PORT_I2C->BRR = PIN_SCL; }
static inline void TWI_SCL_1(void)        { PORT_I2C->BSRR = PIN_SCL; }
static inline void TWI_SDA_0(void)        { PORT_I2C->BRR = PIN_SDA; }
static inline void TWI_SDA_1(void)        { PORT_I2C->BSRR = PIN_SDA; }

static inline uint8_t  TWI_SDA_STATE(void)        { return HAL_GPIO_ReadPin(PORT_I2C, PIN_SDA); }

static const uint8_t  TWI_RETRY_COUNT = 3; //���Դ���  

static void TWI_SendACK(void);
static void TWI_SendNACK(void);


/*******************************************************************************
* ��������:TWI_Delay
* ��    ��:��ʱ����
*
* ��    ��:��
* ��    ��:��
* ��    ��:��
* ��    ��:
* �޸�����:2010��6��8��
*******************************************************************************/
void TWI_NOP(void)
{
	uint32_t i, j;
	static uint32_t sum = 0;
	i = 20;
	while (i--)
	{
		for (j = 0; j < 10; j++)
			sum += i;
	}
	sum = i;
}

/*******************************************************************************
* ��������:TWI_Initialize
* ��    ��:I2C��ʼ������
*
* ��    ��:��
* ��    ��:��
* ��    ��:��
* ��    ��:
* �޸�����:2010��6��8��
*******************************************************************************/
void TWI_Initialize(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	//GPIO_InitStructure.OType = GPIO_OType_OD;
	///GPIO_InitStructure.PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.Pin = PIN_SDA;
	HAL_GPIO_Init(PORT_I2C, &GPIO_InitStructure);

	//GPIO_InitStructure.OType = GPIO_OType_PP;
	GPIO_InitStructure.Pin = PIN_SCL;
	HAL_GPIO_Init(PORT_I2C, &GPIO_InitStructure);

	TWI_SDA_1();
	TWI_SCL_0();
}

/*******************************************************************************
* ��������:TWI_START
* ��    ��:��������
*
* ��    ��:��
* ��    ��:��
* ��    ��:��
* ��    ��:
* �޸�����:2010��6��8��
*******************************************************************************/
uint8_t TWI_START(void)
{
	TWI_SDA_1();
	TWI_NOP();

	TWI_SCL_1();
	TWI_NOP();

	if (!TWI_SDA_STATE())
	{ 
		return TWI_BUS_BUSY;
	}
	TWI_SDA_0();
	TWI_NOP();

	TWI_SCL_0();
	TWI_NOP();

	if (TWI_SDA_STATE())
	{
		return TWI_BUS_ERROR;
	}

	return TWI_READY;
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_START_SHT
*
* @Returns:
*/
/* --------------------------------------------------------------------------*/
uint8_t TWI_START_SHT(void)
{
	TWI_SDA_1();
	TWI_SCL_0();
	TWI_NOP();

	TWI_SDA_1();
	TWI_SCL_1();
	TWI_NOP();

	if (!TWI_SDA_STATE())
	{
		return TWI_BUS_BUSY;
	}
	TWI_SDA_0();
	TWI_NOP();

	TWI_SCL_0();
	TWI_NOP();

	TWI_SCL_1();
	TWI_NOP();

	TWI_SDA_1();
	TWI_NOP();

	TWI_SCL_0();
	TWI_NOP();

	return TWI_READY;
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_STOP
*/
/* --------------------------------------------------------------------------*/
void TWI_STOP(void)
{
	TWI_SDA_0();
	TWI_NOP();

	TWI_SCL_1();
	TWI_NOP();

	TWI_SDA_1();
	TWI_NOP();  
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_SendACK
*/
/* --------------------------------------------------------------------------*/
static void TWI_SendACK(void)
{
	TWI_SDA_0();
	TWI_NOP();
	TWI_SCL_1();
	TWI_NOP();
	TWI_SCL_0();
	TWI_NOP();
	TWI_SDA_1();    
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_SendNACK
*/
/* --------------------------------------------------------------------------*/
static void TWI_SendNACK(void)
{
	TWI_SDA_1();
	TWI_NOP();
	TWI_SCL_1();
	TWI_NOP();
	TWI_SCL_0();
	TWI_NOP();
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_SendByte
*
* @Param: Data
*
* @Returns:
*/
/* --------------------------------------------------------------------------*/
uint8_t TWI_SendByte(uint8_t Data)
{
	uint8_t i;
	TWI_SCL_0();
	for (i = 0; i<8; i++)
	{
		//---------���ݽ���----------  
		if (Data & 0x80)
		{
			TWI_SDA_1();
		}
		else
		{
			TWI_SDA_0();
		}
		Data <<= 1;
		TWI_NOP();
		//---���ݽ�������һ����ʱ----  

		//----����һ��������[������]   
		TWI_SCL_1();
		TWI_NOP();
		TWI_SCL_0();
		TWI_NOP();//��ʱ,��ֹSCL��û��ɵ�ʱ�ı�SDA,�Ӷ�����START/STOP�ź�  
		//---------------------------     
	}
	//���մӻ���Ӧ��   
	TWI_SDA_1();
	TWI_NOP();
	TWI_SCL_1();
	TWI_NOP();
	if (TWI_SDA_STATE())
	{
		TWI_SCL_0();
		TWI_SDA_1();
		//////DebugPrint("TWI_NACK!\n");  
		return TWI_NACK;
	}
	else
	{
		TWI_SCL_0();
		TWI_SDA_1();
		//////DebugPrint("TWI_ACK!\n");  
		return TWI_ACK;
	}
}

/* --------------------------------------------------------------------------*/
/**
* @Brief:  TWI_ReceiveByte
*
* @Returns:
*/
/* --------------------------------------------------------------------------*/
uint8_t TWI_ReceiveByte(void)
{
	uint8_t i, Dat;
	TWI_SDA_1();
	TWI_SCL_0();
	Dat = 0;
	for (i = 0; i<8; i++)
	{
		TWI_SCL_1();//����ʱ��������[������],�ôӻ�׼��������   
		TWI_NOP();
		Dat <<= 1;
		if (TWI_SDA_STATE()) //������״̬  
		{
			Dat |= 0x01;
		}
		TWI_SCL_0();//׼�����ٴν�������    
		TWI_NOP();//�ȴ�����׼����           
	}
	//////DebugPrint("TWI_Dat:%x\n",Dat);  
	return Dat;
}

uint8_t I2CCheck()
{
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */

	__HAL_RCC_GPIOB_CLK_ENABLE();
	  /*Configure GPIO Ϊ����ģʽ */
	GPIO_InitStruct.Pin = PIN_SDA;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PORT_I2C, &GPIO_InitStruct);

	if(TWI_SDA_STATE())
	{
		
	}else
	{
	  	SysLog("SDA is height!");
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pin = PIN_SCL;
		HAL_GPIO_Init(PORT_I2C, &GPIO_InitStruct);

		for(int i=1;i<9;i++)
		{
			TWI_SCL_0();
			
			if(TWI_SDA_STATE())
			{
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
				GPIO_InitStruct.Pin = PIN_SDA;
				HAL_GPIO_Init(PORT_I2C, &GPIO_InitStruct);

				TWI_STOP();
			}else
			{
				TWI_SCL_1();
			}
			TWI_NOP();
		}
		if(TWI_SDA_STATE())
		{
			TWI_STOP();
		}else
		{
			
		}
	}
	return 1;

}
