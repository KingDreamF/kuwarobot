#include "main.h"

static void enterStopMode();
static void enterStandbyMode();

void lowPowerMode()
{
#if 0
  enterStopMode();
#else
  enterStandbyMode();
#endif

}

static void enterStopMode()
{

 	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 3);
  	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  	SysLog("Enter Stop Mode \n"); 
	  
	  
  	//__HAL_RCC_AHB_FORCE_RESET();//io时钟关闭
//	__HAL_RCC_TIM3_CLK_DISABLE();
//	__HAL_RCC_USART1_CLK_DISABLE();
//	__HAL_RCC_I2C1_CLK_DISABLE();
	

	
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
	
	  //低功耗唤醒
	SystemInit();//初始化系统
	SystemClock_Config();//初始化时钟
	
	
	//MX_GPIO_Init();
//	MX_I2C1_Init();
//	MX_USART1_UART_Init();
//	MX_TIM3_Init();

}
static void enterStandbyMode()
{
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
	SysLog("Enter Standby Mode \n"); 
	HAL_PWR_EnterSTANDBYMode();
}