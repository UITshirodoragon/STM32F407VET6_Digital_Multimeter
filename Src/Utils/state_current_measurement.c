/*
 * state_current_measurement.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


#include "state_current_measurement.h"


extern volatile enum measurement_mode current_mode;
extern volatile uint8_t aquisition, hold;

extern volatile float current_value;

static float pre_current_value = 0;




void state_current_enter(void)
{
	HAL_GPIO_WritePin(OF_SEL0_GPIO_Port, OF_SEL0_Pin, GPIO_PIN_RESET); // select ofset is vref/2

	switch (current_mode)
	{
		case MODE_CURRENT_1A:
			HAL_GPIO_WritePin(I_SEL0_GPIO_Port, I_SEL0_Pin, GPIO_PIN_RESET);
			lv_label_set_text_fmt(objects.current_unit_text_label, "A");
			break;
		case MODE_CURRENT_100mA:
			HAL_GPIO_WritePin(I_SEL0_GPIO_Port, I_SEL0_Pin, GPIO_PIN_SET);
			lv_label_set_text_fmt(objects.current_unit_text_label, "mA");
			break;

		default:
			break;
	}

	ADS1220_change_and_config_adc_channel_1_for_current_measurement();

}

void state_current_handle(Event_t event)
{
	   switch (event) {
	        case EVENT_NONE:

	        	ADS1220_read_current_from_adc_channel_1();

	            break;

			case EVENT_CURRENT_VALUE_READY:
				if (hold || pre_current_value == current_value) break;
					        	pre_current_value = current_value;


				switch (current_mode)
				{
					case MODE_CURRENT_1A:
						if (current_value >= 0)
							lv_label_set_text_fmt(objects.current_value_label, "%06.4f", current_value);
						else
							lv_label_set_text_fmt(objects.current_value_label, "%07.4f", current_value);
						break;
						break;
					case MODE_CURRENT_100mA:
						if (current_value >= 0)
							lv_label_set_text_fmt(objects.current_value_label, "%06.2f", current_value);
						else
							lv_label_set_text_fmt(objects.current_value_label, "%07.2f", current_value);
						break;

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
