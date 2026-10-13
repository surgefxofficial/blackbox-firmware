/**
 * @file lv_conf.h
 * LVGL v9 configuration for SurgeFX Black Box.
 * Only widgets/themes actually used by the firmware are enabled.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* clang-format off */
#if 1 /* Set it to "1" to enable the content */

/*====================
 * Graphical settings
 *====================*/
#define LV_COLOR_DEPTH 16

/*====================
 * HAL SETTINGS
 *====================*/
#define LV_DEF_REFR_PERIOD 30 /*[ms]*/

/*====================
 * MEMORY
 *====================*/
/* Default is 64K. The UI is small; bump only if lv_mem_monitor shows pressure. */
#define LV_MEM_SIZE (64U * 1024U)

/*================
 * FONT USAGE
 *================*/
#define LV_FONT_MONTSERRAT_14 1 /* default font */
#define LV_FONT_MONTSERRAT_18 1 /* buttons, titles */
#define LV_FONT_MONTSERRAT_28 1 /* add-preset dialog value */

#define LV_FONT_CUSTOM_DECLARE
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/*==================
 * WIDGETS (only what the firmware uses)
 *================*/
#define LV_USE_ARC        1
#define LV_USE_BUTTON     1
#define LV_USE_IMAGE      1
#define LV_USE_LABEL      1
#if LV_USE_LABEL
  #define LV_LABEL_TEXT_SELECTION 0
  #define LV_LABEL_LONG_TXT_HINT  1
#endif
#define LV_USE_SLIDER     1   /* needs LV_USE_BAR */
#define LV_USE_BAR        1

#define LV_USE_BUTTONMATRIX 0
#define LV_USE_CANVAS     0
#define LV_USE_CHART      0
#define LV_USE_CHECKBOX   0
#define LV_USE_DROPDOWN   0
#define LV_USE_LINE       0
#define LV_USE_ROLLER     0
#define LV_USE_SWITCH     0
#define LV_USE_TEXTAREA   0
#define LV_USE_TABLE      0

/*==================
 * THEMES
 *==================*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
  #define LV_THEME_DEFAULT_DARK 1
  #define LV_THEME_DEFAULT_GROW 1
  #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif
#define LV_USE_THEME_SIMPLE 0
#define LV_USE_THEME_MONO   0

/*==================
 * LAYOUTS
 *==================*/
#define LV_USE_FLEX 1
#define LV_USE_GRID 0

#endif /*End of "Content enable"*/
#endif /*LV_CONF_H*/
