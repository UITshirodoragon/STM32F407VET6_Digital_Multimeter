#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <../lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *voltage_measurement;
    lv_obj_t *current_measurement;
    lv_obj_t *coming_soon;
    lv_obj_t *measurement_mode_roller;
    lv_obj_t *menu_text_label;
    lv_obj_t *voltage_value_label;
    lv_obj_t *voltage_unit_text_label;
    lv_obj_t *current_value_label;
    lv_obj_t *current_unit_text_label;
    lv_obj_t *coming_soon_text_label;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_VOLTAGE_MEASUREMENT = 2,
    SCREEN_ID_CURRENT_MEASUREMENT = 3,
    SCREEN_ID_COMING_SOON = 4,
};

void create_screen_main();
void tick_screen_main();

void create_screen_voltage_measurement();
void tick_screen_voltage_measurement();

void create_screen_current_measurement();
void tick_screen_current_measurement();

void create_screen_coming_soon();
void tick_screen_coming_soon();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/