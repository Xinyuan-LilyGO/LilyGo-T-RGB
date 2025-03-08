/**
 * @file      LV_Helper.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 *
 */
#pragma once
#include <lvgl.h>
#include "LilyGo_Display.h"


void beginLvglHelper(LilyGo_Display &board, bool debug = false);
String lvgl_helper_get_fs_filename(String filename);
const char *lvgl_helper_get_fs_filename(const char *filename);

#if LVGL_VERSION_MAJOR == 8
#define lv_timer_get_user_data(x)       (x->user_data)    
#else
#define LV_USE_PNG  LV_USE_LODEPNG
#define LV_USE_SJPG LV_USE_TJPGD
#endif
