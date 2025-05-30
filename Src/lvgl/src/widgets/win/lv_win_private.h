/**
 * @file lv_win_private.h
 *
 */

#ifndef LV_WIN_PRIVATE_H
#define LV_WIN_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../../../lvgl/src/core/lv_obj_private.h"
#include "../../../../lvgl/src/widgets/win/lv_win.h"

#if LV_USE_WIN

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *      TYPEDEFS
 **********************/
struct _lv_win_t {
    lv_obj_t obj;
};


/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#endif /* LV_USE_WIN */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_WIN_PRIVATE_H*/
