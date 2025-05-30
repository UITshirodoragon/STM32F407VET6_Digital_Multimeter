/*
 * interrupt_manager.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


// interrupt_manager.c
#include "interrupt_manager.h"


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{

	}
	if(htim->Instance == TIM3)
	{

	}
	if(htim->Instance == TIM4)
	{

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case ADS1220_NDRDY_Pin:
			ADS1220_NDRDY_HAL_GPIO_EXTI_Callback();
			break;

		case RE1_BTN_Pin:
			push_event(EVENT_RO_ENC_BTN_1_PUSH);
//			printf("Rotary encoder 1 button\n\r");
			break;
		case RE2_BTN_Pin:
			push_event(EVENT_RO_ENC_BTN_2_PUSH);
//			printf("Rotary encoder 2 button\n\r");
			break;
		case RE3_BTN_Pin:
			push_event(EVENT_RO_ENC_BTN_3_PUSH);
//			printf("Rotary encoder 3 button\n\r");
			break;
		case RED_BTN_Pin:
			push_event(EVENT_BTN_RED_PUSH);
//			printf("Red\n\r");
			break;
		case BLUE_BTN_Pin:
			push_event(EVENT_BTN_BLUE_PUSH);
//			printf("Blue\n\r");
			break;
		case GREEN_BTN_Pin:
			push_event(EVENT_BTN_GREEN_PUSH);
//			printf("Green\n\r");
			break;
		default:
			break;
	}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
    if(hspi->Instance == SPI3)
    {
    	ADS1220_HAL_SPI_RxCpltCallback();
    }
}


