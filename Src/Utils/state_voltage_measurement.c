/*
 * state_voltage_measurement.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


#include "state_voltage_measurement.h"


extern volatile enum measurement_mode current_mode;
extern volatile uint8_t aquisition, hold;
extern volatile float voltage_value;
static float pre_voltage_value = 0;
extern volatile uint16_t samples_count;

extern ADS1220_Handler_t Handler;




void state_voltage_enter(void) {


	HAL_GPIO_WritePin(OF_SEL0_GPIO_Port, OF_SEL0_Pin, GPIO_PIN_RESET); // select ofset is vref/2

	switch (current_mode)
	{
		case MODE_VOLTAGE_10V:
			HAL_GPIO_WritePin(V_SEL0_GPIO_Port, V_SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(V_SEL1_GPIO_Port, V_SEL1_Pin, GPIO_PIN_RESET);
			lv_label_set_text_fmt(objects.voltage_unit_text_label, "V");
			break;

		case MODE_VOLTAGE_1V:
			HAL_GPIO_WritePin(V_SEL0_GPIO_Port, V_SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(V_SEL1_GPIO_Port, V_SEL1_Pin, GPIO_PIN_RESET);
			lv_label_set_text_fmt(objects.voltage_unit_text_label, "V");
			break;

		case MODE_VOLTAGE_100mV:
			HAL_GPIO_WritePin(V_SEL0_GPIO_Port, V_SEL0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(V_SEL1_GPIO_Port, V_SEL1_Pin, GPIO_PIN_SET);
			lv_label_set_text_fmt(objects.voltage_unit_text_label, "mV");
			break;

		case MODE_VOLTAGE_10mV:
			HAL_GPIO_WritePin(V_SEL0_GPIO_Port, V_SEL0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(V_SEL1_GPIO_Port, V_SEL1_Pin, GPIO_PIN_SET);
			lv_label_set_text_fmt(objects.voltage_unit_text_label, "mV");
			break;


		default:
			break;
	}

	ADS1220_change_and_config_adc_channel_2_for_voltage_measurement();




}

void state_voltage_handle(Event_t event) {
    // Xử lý các sự kiện trong chế độ đo điện áp
    switch (event)
    {
        case EVENT_NONE:

        	ADS1220_read_voltage_from_adc_channel_2();
            break;

		case EVENT_VOLTAGE_VALUE_READY:
			if (hold || pre_voltage_value == voltage_value) break;
			        	pre_voltage_value = voltage_value;


			switch (current_mode)
			{

			// nên bổ sung thêm cơ chế khi đo quá điện áp tức là hiện màu đỏ?
				case MODE_VOLTAGE_10V:
					if (voltage_value >= 0)
						lv_label_set_text_fmt(objects.voltage_value_label, "%06.3f", voltage_value);
					else
						lv_label_set_text_fmt(objects.voltage_value_label, "%07.3f", voltage_value);
					break;

				case MODE_VOLTAGE_1V:
					if (voltage_value >= 0)
						lv_label_set_text_fmt(objects.voltage_value_label, "%06.4f", voltage_value);
					else
						lv_label_set_text_fmt(objects.voltage_value_label, "%07.4f", voltage_value);
					break;

				case MODE_VOLTAGE_100mV:
					if (voltage_value >= 0)
						lv_label_set_text_fmt(objects.voltage_value_label, "%06.2f", voltage_value);
					else
						lv_label_set_text_fmt(objects.voltage_value_label, "%07.2f", voltage_value);
					break;

				case MODE_VOLTAGE_10mV:
					if (voltage_value >= 0)
						lv_label_set_text_fmt(objects.voltage_value_label, "%06.3f", voltage_value);
					else
						lv_label_set_text_fmt(objects.voltage_value_label, "%07.3f", voltage_value);
					break;


				default:
					break;
			}
			break;


        case EVENT_RO_ENC_BTN_1_PUSH:
        	ui_fsm_set_screen(SCREEN_ID_MAIN);
        	hold = 0;
        	aquisition = 0;

        	ADS1220_power_down_for_idle();

        	break;

        case EVENT_BTN_RED_PUSH:
			aquisition ^= 1;
			break;

		case EVENT_BTN_BLUE_PUSH:
			hold ^= 1;
			break;

        default:
            break;
    }
}
