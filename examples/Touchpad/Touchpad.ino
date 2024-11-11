/**
 * @file      Touchpad.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-23
 *
 */
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

LilyGo_RGBPanel panel;
lv_obj_t *label;

void setup(void)
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

    label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Touch test");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_obj_center(label);

    const char *touchModel =  panel.getTouchModelName();
    lv_obj_t *model = lv_label_create(lv_scr_act());
    lv_label_set_text(model, touchModel);
    lv_obj_set_style_text_font(model, &lv_font_montserrat_20, 0);
    lv_obj_align_to(model, label, LV_ALIGN_OUT_TOP_MID, 0, 0);

    lv_task_handler();

    panel.setBrightness(16);
}

void loop()
{
    static int16_t x, y;
    bool touched = panel.getPoint(&x, &y);
    if ( touched ) {
        Serial.printf("X:%d Y:%d\n", x, y);
        lv_label_set_text_fmt(label, "X:%d Y:%d", x, y);
        lv_obj_center(label);
    }
    lv_task_handler();
    delay(5);
}

