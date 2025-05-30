/*
 * interrupt_manager.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_INTERRUPT_MANAGER_H_
#define SRC_UTILS_INTERRUPT_MANAGER_H_

#include "event_manager.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "main.h"
#include "../ADS1220_Driver/ADS1220_setup.h"
#include "../ADS1220_Driver/ADS1220.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi);

#endif /* SRC_UTILS_INTERRUPT_MANAGER_H_ */
