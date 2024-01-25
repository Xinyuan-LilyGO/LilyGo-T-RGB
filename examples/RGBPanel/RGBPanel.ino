/**
 * @file      RGBPanel.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 *
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

LilyGo_RGBPanel panel;


extern "C" {
    lv_obj_t *lv_example_meter_1(lv_obj_t *paran);
    lv_obj_t *lv_example_meter_2(lv_obj_t *paran);
    lv_obj_t *lv_example_meter_3(lv_obj_t *paran);
    lv_obj_t *lv_example_meter_4(lv_obj_t *paran);
    void del_example_meter_1();
    void del_example_meter_2();
    void del_example_meter_3();
    void del_example_meter_4();
}


void setup()
{
    Serial.begin(115200);

    // Initialize T-RGB, if the initialization fails, false will be returned.
    if (!panel.begin()) {
        while (1) {
            Serial.println("Error, failed to initialize T-RGB"); delay(1000);
        }
    }
    // Call lvgl initialization
    beginLvglHelper(panel);

    static lv_style_t bgStyle;
    lv_style_init(&bgStyle);
    lv_style_set_bg_color(&bgStyle, lv_color_black());

    lv_obj_t *tileview = lv_tileview_create(lv_scr_act());
    lv_obj_add_style(tileview, &bgStyle, LV_PART_MAIN);
    lv_obj_set_size(tileview, LV_PCT(100), LV_PCT(100));
    // lv_obj_add_event_cb(tileview, tileview_change_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *t1 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t2 = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t3 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t4 = lv_tileview_add_tile(tileview, 3, 0,  LV_DIR_HOR | LV_DIR_BOTTOM);

    lv_example_meter_1(t1);
    lv_example_meter_2(t2);
    lv_example_meter_3(t3);
    lv_example_meter_4(t4);

    lv_timer_handler();

    panel.setBrightness(16);
}


void loop()
{
    lv_timer_handler();
    delay(2);
}

















