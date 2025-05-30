/*
 * state_coming_soon.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


#include "state_coming_soon.h"


void state_coming_soon_enter(void) {

}

void state_coming_soon_handle(Event_t event) {
    // Xử lý các sự kiện trong chế độ đo điện áp
	   switch (event) {
	        case EVENT_NONE:

	            break;
	        case EVENT_RO_ENC_BTN_1_PUSH:
	        	ui_fsm_set_screen(SCREEN_ID_MAIN);
	        	break;

	        default:
	            break;
	    }
}
