/**
 * @file lv_drivers.h
 *
 */

#ifndef LV_DRIVERS_H
#define LV_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../lvgl/src/drivers/sdl/lv_sdl_window.h"
#include "../../../lvgl/src/drivers/sdl/lv_sdl_mouse.h"
#include "../../../lvgl/src/drivers/sdl/lv_sdl_mousewheel.h"
#include "../../../lvgl/src/drivers/sdl/lv_sdl_keyboard.h"

#include "../../../lvgl/src/drivers/x11/lv_x11.h"

#include "../../../lvgl/src/drivers/display/drm/lv_linux_drm.h"
#include "../../../lvgl/src/drivers/display/fb/lv_linux_fbdev.h"

#include "../../../lvgl/src/drivers/display/tft_espi/lv_tft_espi.h"

#include "../../../lvgl/src/drivers/display/lcd/lv_lcd_generic_mipi.h"
#include "../../../lvgl/src/drivers/display/ili9341/lv_ili9341.h"
#include "../../../lvgl/src/drivers/display/st7735/lv_st7735.h"
#include "../../../lvgl/src/drivers/display/st7789/lv_st7789.h"
#include "../../../lvgl/src/drivers/display/st7796/lv_st7796.h"

#include "../../../lvgl/src/drivers/display/renesas_glcdc/lv_renesas_glcdc.h"
#include "../../../lvgl/src/drivers/display/st_ltdc/lv_st_ltdc.h"

#include "../../../lvgl/src/drivers/nuttx/lv_nuttx_entry.h"
#include "../../../lvgl/src/drivers/nuttx/lv_nuttx_fbdev.h"
#include "../../../lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.h"
#include "../../../lvgl/src/drivers/nuttx/lv_nuttx_lcd.h"
#include "../../../lvgl/src/drivers/nuttx/lv_nuttx_libuv.h"

#include "../../../lvgl/src/drivers/evdev/lv_evdev.h"
#include "../../../lvgl/src/drivers/libinput/lv_libinput.h"

#include "../../../lvgl/src/drivers/windows/lv_windows_input.h"
#include "../../../lvgl/src/drivers/windows/lv_windows_display.h"

#include "../../../lvgl/src/drivers/glfw/lv_glfw_window.h"
#include "../../../lvgl/src/drivers/glfw/lv_opengles_texture.h"
#include "../../../lvgl/src/drivers/glfw/lv_opengles_driver.h"

#include "../../../lvgl/src/drivers/qnx/lv_qnx.h"

#include "../../../lvgl/src/drivers/wayland/lv_wayland.h"

#include "../../../lvgl/src/drivers/uefi/lv_uefi_context.h"
#include "../../../lvgl/src/drivers/uefi/lv_uefi_indev.h"
#include "../../../lvgl/src/drivers/uefi/lv_uefi_display.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRIVERS_H*/
