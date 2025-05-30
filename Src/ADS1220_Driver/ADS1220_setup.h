/*
 * ADS1220_setup.h
 *
 *  Created on: May 27, 2025
 *      Author: Admin
 */

#ifndef SRC_ADS1220_DRIVER_ADS1220_SETUP_H_
#define SRC_ADS1220_DRIVER_ADS1220_SETUP_H_

#include "main.h"
#include "../TIM_Delay/TIM_Delay.h"
#include "ADS1220.h"
#include "stdio.h"
#include "math.h"
#include "../Utils/event_manager.h"



// Chip select functions
void CS_UP(void);
void CS_DOWN(void);

// Dummy transmit (needed for DMA read)
void TRANSMIT(uint8_t data);

uint8_t RECEIVE(void);

void DELAY(uint32_t us);

uint8_t TRANSMIT_RECEIVE(uint8_t data);

uint8_t DRDY_Read(void);

void ADS1220_NDRDY_HAL_GPIO_EXTI_Callback(void);

void ADS1220_HAL_SPI_RxCpltCallback(void);


void ADS1220_power_down_for_idle(void);


void ADS1220_change_and_config_adc_channel_1_for_current_measurement(void);


void ADS1220_change_and_config_adc_channel_2_for_voltage_measurement(void);

void ADS1220_read_current_from_adc_channel_1(void);

void ADS1220_read_voltage_from_adc_channel_2(void);





void ADS1220_config_init_start(void);





#endif /* SRC_ADS1220_DRIVER_ADS1220_SETUP_H_ */
