/**
 * @file lvgl.h
 * Include all LVGL related headers
 */

#ifndef LVGL_H
#define LVGL_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************
 * CURRENT VERSION OF LVGL
 ***************************/
#include "../lvgl/lv_version.h"

/*********************
 *      INCLUDES
 *********************/
#include "../lvgl/src/lv_init.h"

#include "../lvgl/src/stdlib/lv_mem.h"
#include "../lvgl/src/stdlib/lv_string.h"
#include "../lvgl/src/stdlib/lv_sprintf.h"

#include "../lvgl/src/misc/lv_log.h"
#include "../lvgl/src/misc/lv_timer.h"
#include "../lvgl/src/misc/lv_math.h"
#include "../lvgl/src/misc/lv_array.h"
#include "../lvgl/src/misc/lv_async.h"
#include "../lvgl/src/misc/lv_anim_timeline.h"
#include "../lvgl/src/misc/lv_profiler_builtin.h"
#include "../lvgl/src/misc/lv_rb.h"
#include "../lvgl/src/misc/lv_utils.h"
#include "../lvgl/src/misc/lv_iter.h"
#include "../lvgl/src/misc/lv_circle_buf.h"
#include "../lvgl/src/misc/lv_tree.h"
#include "../lvgl/src/misc/cache/lv_image_cache.h"

#include "../lvgl/src/tick/lv_tick.h"

#include "../lvgl/src/core/lv_obj.h"
#include "../lvgl/src/core/lv_group.h"
#include "../lvgl/src/indev/lv_indev.h"
#include "../lvgl/src/indev/lv_indev_gesture.h"
#include "../lvgl/src/core/lv_refr.h"
#include "../lvgl/src/display/lv_display.h"

#include "../lvgl/src/font/lv_font.h"
#include "../lvgl/src/font/lv_binfont_loader.h"
#include "../lvgl/src/font/lv_font_fmt_txt.h"

#include "../lvgl/src/widgets/animimage/lv_animimage.h"
#include "../lvgl/src/widgets/arc/lv_arc.h"
#include "../lvgl/src/widgets/bar/lv_bar.h"
#include "../lvgl/src/widgets/button/lv_button.h"
#include "../lvgl/src/widgets/buttonmatrix/lv_buttonmatrix.h"
#include "../lvgl/src/widgets/calendar/lv_calendar.h"
#include "../lvgl/src/widgets/canvas/lv_canvas.h"
#include "../lvgl/src/widgets/chart/lv_chart.h"
#include "../lvgl/src/widgets/checkbox/lv_checkbox.h"
#include "../lvgl/src/widgets/dropdown/lv_dropdown.h"
#include "../lvgl/src/widgets/image/lv_image.h"
#include "../lvgl/src/widgets/imagebutton/lv_imagebutton.h"
#include "../lvgl/src/widgets/keyboard/lv_keyboard.h"
#include "../lvgl/src/widgets/label/lv_label.h"
#include "../lvgl/src/widgets/led/lv_led.h"
#include "../lvgl/src/widgets/line/lv_line.h"
#include "../lvgl/src/widgets/list/lv_list.h"
#include "../lvgl/src/widgets/lottie/lv_lottie.h"
#include "../lvgl/src/widgets/menu/lv_menu.h"
#include "../lvgl/src/widgets/msgbox/lv_msgbox.h"
#include "../lvgl/src/widgets/roller/lv_roller.h"
#include "../lvgl/src/widgets/scale/lv_scale.h"
#include "../lvgl/src/widgets/slider/lv_slider.h"
#include "../lvgl/src/widgets/span/lv_span.h"
#include "../lvgl/src/widgets/spinbox/lv_spinbox.h"
#include "../lvgl/src/widgets/spinner/lv_spinner.h"
#include "../lvgl/src/widgets/switch/lv_switch.h"
#include "../lvgl/src/widgets/table/lv_table.h"
#include "../lvgl/src/widgets/tabview/lv_tabview.h"
#include "../lvgl/src/widgets/textarea/lv_textarea.h"
#include "../lvgl/src/widgets/tileview/lv_tileview.h"
#include "../lvgl/src/widgets/win/lv_win.h"

#include "../lvgl/src/others/snapshot/lv_snapshot.h"
#include "../lvgl/src/others/sysmon/lv_sysmon.h"
#include "../lvgl/src/others/monkey/lv_monkey.h"
#include "../lvgl/src/others/gridnav/lv_gridnav.h"
#include "../lvgl/src/others/fragment/lv_fragment.h"
#include "../lvgl/src/others/imgfont/lv_imgfont.h"
#include "../lvgl/src/others/observer/lv_observer.h"
#include "../lvgl/src/others/ime/lv_ime_pinyin.h"
#include "../lvgl/src/others/file_explorer/lv_file_explorer.h"
#include "../lvgl/src/others/font_manager/lv_font_manager.h"
#include "../lvgl/src/others/xml/lv_xml.h"
#include "../lvgl/src/others/xml/lv_xml_component.h"
#include "../lvgl/src/others/test/lv_test.h"

#include "../lvgl/src/libs/barcode/lv_barcode.h"
#include "../lvgl/src/libs/bin_decoder/lv_bin_decoder.h"
#include "../lvgl/src/libs/bmp/lv_bmp.h"
#include "../lvgl/src/libs/rle/lv_rle.h"
#include "../lvgl/src/libs/fsdrv/lv_fsdrv.h"
#include "../lvgl/src/libs/lodepng/lv_lodepng.h"
#include "../lvgl/src/libs/libpng/lv_libpng.h"
#include "../lvgl/src/libs/gif/lv_gif.h"
#include "../lvgl/src/libs/qrcode/lv_qrcode.h"
#include "../lvgl/src/libs/tjpgd/lv_tjpgd.h"
#include "../lvgl/src/libs/libjpeg_turbo/lv_libjpeg_turbo.h"
#include "../lvgl/src/libs/freetype/lv_freetype.h"
#include "../lvgl/src/libs/rlottie/lv_rlottie.h"
#include "../lvgl/src/libs/ffmpeg/lv_ffmpeg.h"
#include "../lvgl/src/libs/tiny_ttf/lv_tiny_ttf.h"
#include "../lvgl/src/libs/svg/lv_svg.h"
#include "../lvgl/src/libs/svg/lv_svg_render.h"

#include "../lvgl/src/layouts/lv_layout.h"

#include "../lvgl/src/draw/lv_draw_buf.h"
#include "../lvgl/src/draw/lv_draw_vector.h"
#include "../lvgl/src/draw/sw/lv_draw_sw_utils.h"

#include "../lvgl/src/themes/lv_theme.h"

#include "../lvgl/src/drivers/lv_drivers.h"

#include "../lvgl/src/lv_api_map_v8.h"
#include "../lvgl/src/lv_api_map_v9_0.h"
#include "../lvgl/src/lv_api_map_v9_1.h"

#if LV_USE_PRIVATE_API
#include "../lvgl/src/lvgl_private.h"
#endif


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

/** Gives 1 if the x.y.z version is supported in the current version
 * Usage:
 *
 * - Require v6
 * #if LV_VERSION_CHECK(6,0,0)
 *   new_func_in_v6();
 * #endif
 *
 *
 * - Require at least v5.3
 * #if LV_VERSION_CHECK(5,3,0)
 *   new_feature_from_v5_3();
 * #endif
 *
 *
 * - Require v5.3.2 bugfixes
 * #if LV_VERSION_CHECK(5,3,2)
 *   bugfix_in_v5_3_2();
 * #endif
 *
 */
#define LV_VERSION_CHECK(x,y,z) (x == LVGL_VERSION_MAJOR && (y < LVGL_VERSION_MINOR || (y == LVGL_VERSION_MINOR && z <= LVGL_VERSION_PATCH)))

/**
 * Wrapper functions for VERSION macros
 */

static inline int lv_version_major(void)
{
    return LVGL_VERSION_MAJOR;
}

static inline int lv_version_minor(void)
{
    return LVGL_VERSION_MINOR;
}

static inline int lv_version_patch(void)
{
    return LVGL_VERSION_PATCH;
}

static inline const char * lv_version_info(void)
{
    return LVGL_VERSION_INFO;
}

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LVGL_H*/
