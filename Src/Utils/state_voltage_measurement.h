/*
 * state_voltage_measurement.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_STATE_VOLTAGE_MEASUREMENT_H_
#define SRC_UTILS_STATE_VOLTAGE_MEASUREMENT_H_

#include "event_manager.h"
#include "../lvgl/lvgl.h"
#include "../ui/ui.h"
#include "../ui/screens.h"
#include "ui_fsm_manager.h"
#include "main.h"
#include "../ADS1220_Driver/ADS1220.h"
#include "../ADS1220_Driver/ADS1220_setup.h"

void state_voltage_enter(void);
void state_voltage_handle(Event_t event);

#endif /* SRC_UTILS_STATE_VOLTAGE_MEASUREMENT_H_ */
