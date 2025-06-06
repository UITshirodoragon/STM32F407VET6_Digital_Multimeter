/**
 * @file lv_xml_parser.h
 *
 */

#ifndef LV_XML_PARSER_H
#define LV_XML_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../../../lvgl/src/misc/lv_types.h"
#include "../../../../lvgl/src/misc/lv_style.h"
#if LV_USE_XML

#include "../../../../lvgl/src/others/xml/lv_xml_component.h"
#include "../../../../lvgl/src/others/xml/lv_xml_component_private.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

typedef enum {
    LV_XML_PARSER_SECTION_NONE,
    LV_XML_PARSER_SECTION_API,
    LV_XML_PARSER_SECTION_CONSTS,
    LV_XML_PARSER_SECTION_GRAD,
    LV_XML_PARSER_SECTION_GRAD_STOP,
    LV_XML_PARSER_SECTION_STYLES,
    LV_XML_PARSER_SECTION_FONTS,
    LV_XML_PARSER_SECTION_IMAGES,
    LV_XML_PARSER_SECTION_SUBJECTS,
    LV_XML_PARSER_SECTION_VIEW
} lv_xml_parser_section_t;

struct _lv_xml_parser_state_t {
    lv_xml_component_ctx_t ctx;
    lv_ll_t parent_ll;
    lv_obj_t * parent;
    lv_obj_t * item;
    lv_obj_t * view;    /*Pointer to the created view during component creation*/
    const char ** parent_attrs;
    lv_xml_component_ctx_t * parent_ctx;
    lv_xml_parser_section_t section;
};

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_xml_parser_state_init(lv_xml_parser_state_t * state);

void lv_xml_parser_start_section(lv_xml_parser_state_t * state, const char * name);

void lv_xml_parser_end_section(lv_xml_parser_state_t * state, const char * name);

void * lv_xml_state_get_parent(lv_xml_parser_state_t * state);

void * lv_xml_state_get_item(lv_xml_parser_state_t * state);

/**********************
 *      MACROS
 **********************/

#endif /* LV_USE_XML */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_XML_PARSER_H*/
