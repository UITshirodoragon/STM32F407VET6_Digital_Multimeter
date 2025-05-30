/**
 * @file lv_vg_lite_math.h
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "../../../../lvgl/src/draw/vg_lite/lv_vg_lite_math.h"

#if LV_USE_DRAW_VG_LITE

#include <stdint.h>

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

float math_fast_inv_sqrtf(float number)
{
    const float threehalfs = 1.5f;

    float x2 = number * 0.5f;
    float y = number;
    int32_t i = *(int32_t *)&y; /* evil floating point bit level hacking */

    i = 0x5f3759df /* floating-point representation of an approximation of {\sqrt {2^{127}}}} see https://en.wikipedia.org/wiki/Fast_inverse_square_root. */
        - (i >>
           1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y)); /* 1st iteration */

    return y;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

#endif /*LV_USE_DRAW_VG_LITE*/
