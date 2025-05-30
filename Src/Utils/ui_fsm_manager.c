/*
 * ui_fsm_manager.c
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */


// ui_fsm.c
# include "ui_fsm_manager.h"

volatile enum ScreensEnum current_screen = SCREEN_ID_MAIN;
volatile enum measurement_mode current_mode = MODE_VOLTAGE_10V;

volatile uint8_t aquisition, hold;



void ui_fsm_init(void) {
    current_screen = SCREEN_ID_MAIN;
    state_menu_enter();
}

void ui_fsm_update(void) {
    Event_t evt = pop_event();
    switch (current_screen) {
    		case SCREEN_ID_VOLTAGE_MEASUREMENT:
                state_voltage_handle(evt);
                break;
            case SCREEN_ID_CURRENT_MEASUREMENT:
            	state_current_handle(evt);
                break;
            case SCREEN_ID_MAIN:
            	state_menu_handle(evt);
                break;
            case SCREEN_ID_COMING_SOON:
            	state_coming_soon_handle(evt);
    			break;
        default:
            break;
    }
}

void ui_fsm_set_screen(enum ScreensEnum screen) {
    if(current_screen == screen) return;

    current_screen = screen;
    switch (screen) {
        case SCREEN_ID_VOLTAGE_MEASUREMENT:
            state_voltage_enter();
            break;
        case SCREEN_ID_CURRENT_MEASUREMENT:
            state_current_enter();
            break;
        case SCREEN_ID_MAIN:
            state_menu_enter();
            break;
        case SCREEN_ID_COMING_SOON:
			state_coming_soon_enter();
			break;
        default:
            break;
    }
    loadScreen(screen);
}
enum ScreensEnum ui_fsm_get_screen(void) {
    return current_screen;
}
