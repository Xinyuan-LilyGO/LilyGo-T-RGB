/**
 * @file      lv_helloworld.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-27
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

LilyGo_RGBPanel panel;

static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;
        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
        Serial.printf("Button :%d\n", cnt);
    }
}

void setup()
{
    Serial.begin(115200);


    //** Four initialization methods */

    // Automatically determine the touch model to determine the initialization screen type. If touch is not available, it may fail.
    bool rslt = panel.begin();

    // Specify 2.1-inch semicircular screen
    // https://www.lilygo.cc/products/t-rgb?variant=42407295877301
    // bool rslt = panel.begin(LILYGO_T_RGB_2_1_INCHES_HALF_CIRCLE);

    // Specified as a 2.1-inch full-circle screen
    // https://www.lilygo.cc/products/t-rgb
    // bool rslt = panel.begin(LILYGO_T_RGB_2_1_INCHES_FULL_CIRCLE);

    // Specified as a 2.8-inch full-circle screen
    // https://www.lilygo.cc/products/t-rgb?variant=42880799441077
    // bool rslt = panel.begin(LILYGO_T_RGB_2_8_INCHES);

    if (!rslt) {
        while (1) {
            Serial.println("Error, failed to initialize T-RGB"); delay(1000);
        }
    }

    // Call lvgl initialization
    beginLvglHelper(panel);


    lv_obj_t *label = lv_label_create(lv_scr_act());        /*Add a label the current screen*/
    lv_label_set_text(label, "Hello World");                 /*Set label text*/
    lv_obj_center(label);                                   /*Set center alignment*/


    lv_obj_t *btn = lv_btn_create(lv_scr_act());            /*Add a button the current screen*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);  /*Assign a callback to the button*/
    lv_obj_align_to(btn, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 0); /*Set the label to it and align it in the center below the label*/

    lv_obj_t *btn_label = lv_label_create(btn);           /*Add a label to the button*/
    lv_label_set_text(btn_label, "Button");               /*Set the labels text*/
    lv_obj_center(btn_label);

    // Turn on the backlight and set it to the highest value, ranging from 0 to 16
    panel.setBrightness(16);
}


void loop()
{
    // lvgl task processing should be placed in the loop function
    lv_timer_handler();
    delay(2);
}

















