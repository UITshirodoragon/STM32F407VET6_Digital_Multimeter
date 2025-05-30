/*
 * ui_fsm_manager.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_UI_FSM_MANAGER_H_
#define SRC_UTILS_UI_FSM_MANAGER_H_

#include "event_manager.h"
#include "../lvgl/lvgl.h"
#include "../ui/ui.h"
#include "../ui/screens.h"
#include "state_voltage_measurement.h"
#include "state_coming_soon.h"
#include "state_current_measurement.h"
#include "state_menu.h"


void ui_fsm_init(void);
void ui_fsm_update(void);
void ui_fsm_set_screen(enum ScreensEnum screen);
enum ScreensEnum ui_fsm_get_screen(void);



#endif /* SRC_UTILS_UI_FSM_MANAGER_H_ */
