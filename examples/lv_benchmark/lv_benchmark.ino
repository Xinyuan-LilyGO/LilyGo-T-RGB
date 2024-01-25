/**
 * @file      lv_benchmark.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 *
 */
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include "lv_demo_benchmark.h"

LilyGo_RGBPanel panel;

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

    lv_demo_benchmark();

    panel.setBrightness(16);
}

void loop()
{
    lv_timer_handler();
    delay(2);
}
