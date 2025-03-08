/**
 * @file      LV_Helper_v9.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2025  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2025-03-08
 * @note      Adapt to lvgl 9 version
 */
#include <Arduino.h>
#include "LV_Helper.h"

#if LVGL_VERSION_MAJOR == 9

static lv_display_t *disp_drv;
static lv_draw_buf_t draw_buf;
static lv_indev_t *indev_drv;

static lv_color16_t *buf  = NULL;
static lv_color16_t *buf1  = NULL;

static void disp_flush( lv_display_t *disp_drv, const lv_area_t *area, uint8_t *color_p)
{
    auto *plane = (LilyGo_Display *)lv_display_get_user_data(disp_drv);
    assert(plane);
    plane->pushColors(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *)color_p);
    lv_display_flush_ready( disp_drv );
}

/*Read the touchpad*/
static void touchpad_read( lv_indev_t *indev, lv_indev_data_t *data )
{
    static int16_t x, y;
    auto *plane = (LilyGo_Display *)lv_indev_get_user_data(indev);
    assert(plane);
    uint8_t touched = plane->getPoint(&x, &y, 1);
    if ( touched ) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PR;
        return;
    }
    data->state = LV_INDEV_STATE_REL;
}

static uint32_t lv_tick_get_cb(void)
{
    return millis();
}

String lvgl_helper_get_fs_filename(String filename)
{
    static String path;
    path = String(LV_FS_POSIX_LETTER) + ":" + (filename);
    return path;
}

const char *lvgl_helper_get_fs_filename(const char *filename)
{
    static String path;
    path = String(LV_FS_POSIX_LETTER) + ":" + String(filename);
    return path.c_str();
}

void beginLvglHelper(LilyGo_Display &board, bool debug)
{

    lv_init();

    size_t lv_buffer_size = board.width() * board.height() * sizeof(lv_color16_t);

    buf = (lv_color16_t *)ps_malloc(lv_buffer_size);
    assert(buf);

    buf1 = (lv_color16_t *)ps_malloc(lv_buffer_size);
    assert(buf1);

    disp_drv = lv_display_create(board.width(), board.height());

    lv_display_set_buffers(disp_drv, buf, buf1, lv_buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_color_format(disp_drv, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp_drv, disp_flush);
    lv_display_set_user_data(disp_drv, &board);

    indev_drv = lv_indev_create();
    lv_indev_set_type(indev_drv, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_drv, touchpad_read);
    lv_indev_set_user_data(indev_drv, &board);
    lv_indev_enable(indev_drv, true);
    lv_indev_set_display(indev_drv, disp_drv);

    lv_tick_set_cb(lv_tick_get_cb);

    lv_group_set_default(lv_group_create());
}


extern "C" void lv_mem_init(void)
{
    return; /*Nothing to init*/
}

extern "C" void lv_mem_deinit(void)
{
    return; /*Nothing to deinit*/

}

extern "C" lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    /*Not supported*/
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

extern "C" void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    /*Not supported*/
    LV_UNUSED(pool);
    return;
}

extern "C" void *lv_malloc_core(size_t size)
{
    return ps_malloc(size);
}

extern "C" void *lv_realloc_core(void *p, size_t new_size)
{
    return ps_realloc(p, new_size);
}

extern "C" void lv_free_core(void *p)
{
    free(p);
}

extern "C" void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    /*Not supported*/
    LV_UNUSED(mon_p);
    return;
}

lv_result_t lv_mem_test_core(void)
{
    /*Not supported*/
    return LV_RESULT_OK;
}
#endif
