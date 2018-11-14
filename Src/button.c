#include "button.h"
static uint8_t key_time =0;
void keyScan()
{
	
}
uint8_t last_s =0;
void buttonOperation()
{
//  uint8_t status;
//  // = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
// HAL_Delay(30);
//  status = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
////  if(last_s==0)
//  printf(" %d\n",status);
////  if(last_s!=status)
////  {
////  	printf("%d\n",status);
////	last_s=status;
////  }else
////  {
////  
////  }
////  
	if(button.flage==DOWN)
	{
	  	if(button.down_time==0)
	  		button.down_time = HAL_GetTick();
		
		if(abs(HAL_GetTick()-button.down_time)>=COUNTTIME)
		{
			button.long_press = 1;
			//button.down_time=0;
			//button.flage=0;
			printf("long_press\n");
		}
	  
	}else if(button.flage==UP)
	{
  		if(abs(HAL_GetTick()-button.down_time)<=COUNTTIME)
  		{
	  		button.short_press = 1;
			button.down_time=0;
			printf("short_press\n");
		}
		
		button.flage=0;
	}
}