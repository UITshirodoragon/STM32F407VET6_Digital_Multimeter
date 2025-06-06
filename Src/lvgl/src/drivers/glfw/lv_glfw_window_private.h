/**
 * @file lv_glfw_window_private.h
 *
 */

#ifndef LV_GLFW_WINDOW_PRIVATE_H
#define LV_GLFW_WINDOW_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../../../lvgl/src/drivers/glfw/lv_glfw_window.h"
#if LV_USE_OPENGLES

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../../../../lvgl/src/misc/lv_area.h"
#include "../../../../lvgl/src/display/lv_display.h"
#include "../../../../lvgl/src/indev/lv_indev.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

struct _lv_glfw_window_t {
    GLFWwindow * window;
    int32_t hor_res;
    int32_t ver_res;
    lv_ll_t textures;
    lv_point_t mouse_last_point;
    lv_indev_state_t mouse_last_state;
    uint8_t use_indev : 1;
    uint8_t closing : 1;
};

struct _lv_glfw_texture_t {
    lv_glfw_window_t * window;
    unsigned int texture_id;
    lv_area_t area;
    lv_opa_t opa;
    lv_indev_t * indev;
    lv_point_t indev_last_point;
    lv_indev_state_t indev_last_state;
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_OPENGLES*/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_GLFW_WINDOW_PRIVATE_H*/
