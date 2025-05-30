/*
 * state_menu.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


#include "state_menu.h"


volatile uint32_t selected_index = 0;
static uint32_t current_rotary_encoder_1_value = 0;

extern volatile enum measurement_mode current_mode;


void state_menu_enter(void) {
	current_rotary_encoder_1_value = TIM2->CNT;
}

void state_menu_handle(Event_t event) {
    switch (event)
    {
        case EVENT_NONE:
            // xoay encode để chọn
        	if (current_rotary_encoder_1_value == TIM2->CNT) break;

        	current_rotary_encoder_1_value = TIM2->CNT;

        	lv_roller_set_selected(objects.measurement_mode_roller, ((TIM2->CNT)/2), LV_ANIM_ON);
            break;

        case EVENT_RO_ENC_BTN_1_PUSH:
            selected_index = lv_roller_get_selected(objects.measurement_mode_roller);


            current_mode = selected_index;

            switch (current_mode)
            {
				case MODE_VOLTAGE_10V:
				case MODE_VOLTAGE_1V:
				case MODE_VOLTAGE_100mV:
				case MODE_VOLTAGE_10mV:
					ui_fsm_set_screen(SCREEN_ID_VOLTAGE_MEASUREMENT);

					break;
				case MODE_CURRENT_1A:
				case MODE_CURRENT_100mA:

					ui_fsm_set_screen(SCREEN_ID_CURRENT_MEASUREMENT);

					break;

				default:

					ui_fsm_set_screen(SCREEN_ID_COMING_SOON);
            		break;
            }

            break;
        default:
            break;
    }
}
