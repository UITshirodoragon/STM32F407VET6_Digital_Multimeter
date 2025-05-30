/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_BTN_Pin GPIO_PIN_3
#define RED_BTN_GPIO_Port GPIOE
#define RED_BTN_EXTI_IRQn EXTI3_IRQn
#define BLUE_BTN_Pin GPIO_PIN_4
#define BLUE_BTN_GPIO_Port GPIOE
#define BLUE_BTN_EXTI_IRQn EXTI4_IRQn
#define GREEN_BTN_Pin GPIO_PIN_5
#define GREEN_BTN_GPIO_Port GPIOE
#define GREEN_BTN_EXTI_IRQn EXTI9_5_IRQn
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOE
#define RE1_CH1_Pin GPIO_PIN_0
#define RE1_CH1_GPIO_Port GPIOA
#define RE1_CH2_Pin GPIO_PIN_1
#define RE1_CH2_GPIO_Port GPIOA
#define RE1_BTN_Pin GPIO_PIN_2
#define RE1_BTN_GPIO_Port GPIOA
#define RE1_BTN_EXTI_IRQn EXTI2_IRQn
#define O_ADC0_Pin GPIO_PIN_3
#define O_ADC0_GPIO_Port GPIOA
#define O_ADC1_Pin GPIO_PIN_4
#define O_ADC1_GPIO_Port GPIOA
#define RE2_BTN_Pin GPIO_PIN_6
#define RE2_BTN_GPIO_Port GPIOA
#define RE2_BTN_EXTI_IRQn EXTI9_5_IRQn
#define RE3_BTN_Pin GPIO_PIN_7
#define RE3_BTN_GPIO_Port GPIOA
#define RE3_BTN_EXTI_IRQn EXTI9_5_IRQn
#define LCD_BL_Pin GPIO_PIN_1
#define LCD_BL_GPIO_Port GPIOB
#define LCD_NRST_Pin GPIO_PIN_13
#define LCD_NRST_GPIO_Port GPIOD
#define RE2_CH1_Pin GPIO_PIN_6
#define RE2_CH1_GPIO_Port GPIOC
#define RE2_CH2_Pin GPIO_PIN_7
#define RE2_CH2_GPIO_Port GPIOC
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define ADS1220_NCS_Pin GPIO_PIN_15
#define ADS1220_NCS_GPIO_Port GPIOA
#define ADS1220_SPI_SCK_Pin GPIO_PIN_10
#define ADS1220_SPI_SCK_GPIO_Port GPIOC
#define ADS1220_SPI_MISO_Pin GPIO_PIN_11
#define ADS1220_SPI_MISO_GPIO_Port GPIOC
#define ADS1220_SPI_MOSI_Pin GPIO_PIN_12
#define ADS1220_SPI_MOSI_GPIO_Port GPIOC
#define O_SEL0_Pin GPIO_PIN_2
#define O_SEL0_GPIO_Port GPIOD
#define OF_SEL0_Pin GPIO_PIN_5
#define OF_SEL0_GPIO_Port GPIOB
#define RE3_CH1_Pin GPIO_PIN_6
#define RE3_CH1_GPIO_Port GPIOB
#define RE3_CH2_Pin GPIO_PIN_7
#define RE3_CH2_GPIO_Port GPIOB
#define I_SEL0_Pin GPIO_PIN_8
#define I_SEL0_GPIO_Port GPIOB
#define V_SEL1_Pin GPIO_PIN_9
#define V_SEL1_GPIO_Port GPIOB
#define V_SEL0_Pin GPIO_PIN_0
#define V_SEL0_GPIO_Port GPIOE
#define ADS1220_NDRDY_Pin GPIO_PIN_1
#define ADS1220_NDRDY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
