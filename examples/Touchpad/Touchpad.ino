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

    // Initialize T-RGB, if the initialization fails, false will be returned.
    if (!panel.begin()) {
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

