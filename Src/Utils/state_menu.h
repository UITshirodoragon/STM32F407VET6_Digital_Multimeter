/*
 * state_menu.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_STATE_MENU_H_
#define SRC_UTILS_STATE_MENU_H_

#include "event_manager.h"
#include "../lvgl/lvgl.h"
#include "../ui/ui.h"
#include "../ui/screens.h"
#include "ui_fsm_manager.h"
#include "main.h"

enum measurement_mode
{
	MODE_VOLTAGE_10V = 0,
	MODE_VOLTAGE_1V,
	MODE_VOLTAGE_100mV,
	MODE_VOLTAGE_10mV,
	MODE_CURRENT_1A,
	MODE_CURRENT_100mA,
	MODE_CONTINUTY,
	MODE_RESISTOR,
	MODE_CAPACITOR,
	MODE_DIODE,
	MODE_FREQUENCY
};

void state_menu_enter(void);
void state_menu_handle(Event_t event);

#endif /* SRC_UTILS_STATE_MENU_H_ */
