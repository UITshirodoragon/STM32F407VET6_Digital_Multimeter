/*
 * state_comming_soon.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_STATE_COMING_SOON_H_
#define SRC_UTILS_STATE_COMING_SOON_H_

#include "event_manager.h"
#include "../lvgl/lvgl.h"
#include "../ui/ui.h"
#include "../ui/screens.h"
#include "ui_fsm_manager.h"
#include "main.h"

void state_coming_soon_enter(void);
void state_coming_soon_handle(Event_t event);

#endif /* SRC_UTILS_STATE_COMING_SOON_H_ */
