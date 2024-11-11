/**
 * @file      lv_qrcode.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

#if !LV_USE_QRCODE
#error "lvgl qrcode library is not enable!"
#endif


LilyGo_RGBPanel panel;

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

    lv_color_t bg_color = lv_color_white();//lv_palette_lighten(LV_PALETTE_LIGHT_BLUE, 5);
    lv_color_t fg_color = lv_color_black();//lv_palette_darken(LV_PALETTE_BLUE, 4);

    lv_obj_t *qr = lv_qrcode_create(lv_scr_act(), 200, fg_color, bg_color);

    /*Set data*/
    const char *data = "https://www.lilygo.cc/";
    lv_qrcode_update(qr, data, strlen(data));
    lv_obj_center(qr);

    /*Add a border with bg_color*/
    lv_obj_set_style_border_color(qr, bg_color, 0);
    lv_obj_set_style_border_width(qr, 5, 0);


    panel.setBrightness(16);
}


void loop()
{
    lv_timer_handler();
    delay(2);
}

















