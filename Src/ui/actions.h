#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <../lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_open_menu(lv_event_t * e);
extern void action_open_menu_close_menu(lv_event_t * e);
extern void action_open_menu_close_menu_select_mode(lv_event_t * e);
extern void action_close_menu_select_mode(lv_event_t * e);
extern void action_change_measurement_mode(lv_event_t * e);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/