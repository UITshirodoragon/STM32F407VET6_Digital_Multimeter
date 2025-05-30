#include "TIM_Delay.h"

uint32_t gu32_ticks = 0;
TIM_HandleTypeDef HTIMx;
void timDelayInit()
{


    // timer 6 init form deepblue
    gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	HTIMx.Instance = TIMER;
	HTIMx.Init.Prescaler = gu32_ticks-1;
	HTIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
	HTIMx.Init.Period = 65535;

	HTIMx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&HTIMx) != HAL_OK)
	{
	  Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&HTIMx, &sClockSourceConfig) != HAL_OK)
	{
	  Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&HTIMx, &sMasterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_TIM_Base_Start(&HTIMx);
}
