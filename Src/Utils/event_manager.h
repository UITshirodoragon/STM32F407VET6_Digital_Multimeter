/*
 * event_manager.h
 *
 *  Created on: May 24, 2025
 *      Author: Admin
 */

#ifndef SRC_UTILS_EVENT_MANAGER_H_
#define SRC_UTILS_EVENT_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

// Định nghĩa các loại sự kiện có thể xảy ra trong hệ thống
typedef enum {
    EVENT_NONE = 0,
    EVENT_1MS_TICK,
    EVENT_LVGL_TICK,
    EVENT_MEAS_REQ,
    EVENT_RO_ENC_BTN_1_PUSH,
	EVENT_RO_ENC_BTN_2_PUSH,
	EVENT_RO_ENC_BTN_3_PUSH,
	EVENT_RO_ENC_1_CNT_UP,
	EVENT_RO_ENC_2_CNT_UP,
	EVENT_RO_ENC_3_CNT_UP,
	EVENT_RO_ENC_1_CNT_DOWN,
	EVENT_RO_ENC_2_CNT_DOWN,
	EVENT_RO_ENC_3_CNT_DOWN,
    EVENT_BTN_RED_PUSH,
    EVENT_BTN_BLUE_PUSH,
    EVENT_BTN_GREEN_PUSH,
    EVENT_ADC_READY,
	EVENT_VOLTAGE_VALUE_READY,
	EVENT_CURRENT_VALUE_READY,
    EVENT_MAX
} Event_t;



// Hàm khởi tạo hệ thống quản lý sự kiện
void event_manager_init(void);

// Thêm một sự kiện vào hàng đợi
bool push_event(Event_t event);

// Lấy một sự kiện ra khỏi hàng đợi
Event_t pop_event(void);

#endif /* SRC_UTILS_EVENT_MANAGER_H_ */
