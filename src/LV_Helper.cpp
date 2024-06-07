/**
 * @file      LV_Helper.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 *
 */
#include <Arduino.h>
#include "LV_Helper.h"


#if LV_VERSION_CHECK(9,0,0)
//#error "Currently not supported 9.x"
#endif

static lv_draw_buf_t draw_buf;
static lv_display_t * disp_drv;
static lv_indev_t * indev_drv;
static lv_color_t *buf = NULL;
static lv_color_t *buf1 = NULL;

/* Display flushing */
static void disp_flush( lv_disp_t *disp_drv, const lv_area_t *area, uint8_t * px_map)
{
    auto *board = (LilyGo_Display *)lv_display_get_user_data(disp_drv);
    // rotate content by 180 degrees - TODO
    int pos_x = area->x1;
    int pos_y = area->y1;
    int width = area->x2;
    int height = area->y2;
    ESP_LOGV("LVGL", "flushing area: x1=%d, y1=%d, x2=%d, y2=%d", pos_x, pos_y, width+1, height+1);
    board->pushColors(area->x1, area->y1, area->x2 + 1, area->y2 + 1, (uint16_t *)px_map);
    lv_display_flush_ready( disp_drv );
}

/*Read the touchpad*/
static void touchpad_read( lv_indev_t *indev_driver, lv_indev_data_t *data )
{
    Serial.print("touchpad_read");
    static int16_t x, y;
    auto *board = (LilyGo_Display *)lv_indev_get_user_data(indev_driver);
    uint8_t touched =  board->getPoint(&x, &y, 1);
    if ( touched ) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PR;
        Serial.printf(": x=%d, y=%d\n", x, y);
        return;
    }
    data->state = LV_INDEV_STATE_REL;
}

#if LV_USE_LOG
void lv_log_print_g_cb(lv_log_level_t level, const char *buf)
{
    switch (level)
    {
    case LV_LOG_LEVEL_WARN:
        ESP_LOGW("LVGL", "%s", buf);    
        break;
    case LV_LOG_LEVEL_INFO:
        ESP_LOGI("LVGL", "%s", buf);
        break;
    case LV_LOG_LEVEL_ERROR:
        ESP_LOGE("LVGL", "%s", buf);
        break;
    case LV_LOG_LEVEL_TRACE:
        ESP_LOGV("LVGL", "%s", buf);
        break;
    case LV_LOG_LEVEL_USER:
        ESP_LOGW("LVGL", "%s", buf);
        break;
    default:
        ESP_LOGD("LVGL", "%s", buf);
    }
}
#endif

String lvgl_helper_get_fs_filename(String filename)
{
    static String path;
    // path = String(LV_FS_POSIX_LETTER) + ":" + (filename);
    return path;
}

const char *lvgl_helper_get_fs_filename(const char *filename)
{
    static String path;
    // path = String(LV_FS_POSIX_LETTER) + ":" + String(filename);
    return path.c_str();
}

void beginLvglHelper(LilyGo_Display &board, bool debug)
{

    lv_init();

#if LV_USE_LOG
    if (debug) {
        lv_log_register_print_cb(lv_log_print_g_cb);
    }
#endif

    size_t lv_buffer_size = 480 * 480 * sizeof(lv_color_t);
    buf = (lv_color_t *)ps_malloc(lv_buffer_size);
    assert(buf);
    buf1 = (lv_color_t *)ps_malloc(lv_buffer_size);
    assert(buf1);

    disp_drv = lv_display_create(480,480);
    lv_display_set_buffers(disp_drv, buf, buf1, lv_buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp_drv, disp_flush);
    lv_display_set_user_data(disp_drv, &board);

    indev_drv = lv_indev_create();
    lv_indev_set_type(indev_drv, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_drv, touchpad_read);
    lv_indev_set_user_data(indev_drv, &board);
}
