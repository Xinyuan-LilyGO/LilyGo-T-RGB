/**
 * @file      TFT_eSPI_Sprite.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-23
 * @note      Use TFT_eSPI Sprite made by framebuffer , unnecessary calling during use tft.xxxx function
 */


#include "esp_arduino_version.h"

#if ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3,0,0)

#include <TFT_eSPI.h>   //https://github.com/Bodmer/TFT_eSPI
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
LilyGo_RGBPanel panel;

#define WIDTH  panel.width()
#define HEIGHT panel.height()

void setup()
{
    Serial.begin(115200);


    //！ Use TFT_eSPI Sprite made by framebuffer , unnecessary calling during use tft.xxxx function
    //！ Use TFT_eSPI sprites requires exchanging the color order

    // Automatically determine the touch model to determine the initialization screen type. If touch is not available, it may fail.
    bool rslt = panel.begin(LILYGO_T_RGB_ORDER_BGR);

    // Specify 2.1-inch semicircular screen
    // https://www.lilygo.cc/products/t-rgb?variant=42407295877301
    // bool rslt = panel.begin(LILYGO_T_RGB_2_1_INCHES_HALF_CIRCLE,LILYGO_T_RGB_ORDER_BGR);

    // Specified as a 2.1-inch full-circle screen
    // https://www.lilygo.cc/products/t-rgb
    // bool rslt = panel.begin(LILYGO_T_RGB_2_1_INCHES_FULL_CIRCLE,LILYGO_T_RGB_ORDER_BGR);

    // Specified as a 2.8-inch full-circle screen
    // https://www.lilygo.cc/products/t-rgb?variant=42880799441077
    // bool rslt = panel.begin(LILYGO_T_RGB_2_8_INCHES,LILYGO_T_RGB_ORDER_BGR);

    if (!rslt) {
        while (1) {
            Serial.println("Error, failed to initialize T-RGB"); delay(1000);
        }
    }
    spr.createSprite(WIDTH, HEIGHT);

    spr.setColorDepth(16);

    spr.setSwapBytes(1);

    panel.setBrightness(16);

}

void loop()
{

    int x = WIDTH / 2;
    spr.setTextDatum(MC_DATUM);

    spr.fillSprite(TFT_RED);
    spr.setTextColor(TFT_WHITE, TFT_RED);
    spr.drawString("Red Color", x, HEIGHT / 2, 4);
    panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
    delay(2000);

    spr.fillSprite(TFT_GREEN);
    spr.setTextColor(TFT_WHITE, TFT_GREEN);
    spr.drawString("Green Color", x, HEIGHT / 2, 4);
    panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
    delay(2000);

    spr.fillSprite(TFT_BLUE);
    spr.setTextColor(TFT_WHITE, TFT_BLUE);
    spr.drawString("Blue Color", x, HEIGHT / 2, 4);
    panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
    delay(2000);

    spr.fillSprite(TFT_BLACK);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString("Black Color", x - 20, HEIGHT / 2, 4);
    panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
    delay(2000);

    uint16_t colors[6] = {TFT_RED, TFT_GREEN, TFT_BLUE, TFT_YELLOW, TFT_CYAN, TFT_MAGENTA};
    for (int i = 0; i < 6; ++i) {
        spr.fillSprite(TFT_BLACK);
        spr.setTextColor(colors[i], TFT_BLACK);
        spr.drawString("LilyGo.cc", WIDTH / 2, HEIGHT / 2, 4);
        spr.drawString("T-RGB", WIDTH / 2, HEIGHT / 2 + 30, 4);
        panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
        delay(200);
    }
    delay(2000);


    for (int pos = WIDTH; pos > 0; pos--) {
        int h = HEIGHT;
        while (h--) spr.drawFastHLine(0, h, WIDTH, rainbow(h * 4));
        spr.setTextSize(1);
        spr.setTextFont(4);
        spr.setTextColor(TFT_WHITE);
        spr.setTextWrap(false);
        spr.setCursor(pos, 100);
        spr.print("LilyGo LilyGo LilyGo");
        panel.pushColors(0, 0, WIDTH, HEIGHT, (uint16_t *)spr.getPointer());
    }
    delay(2000);
}





// #########################################################################
// Return a 16 bit rainbow colour
// #########################################################################
unsigned int rainbow(uint8_t value)
{
    // Value is expected to be in range 0-127
    // The value is converted to a spectrum colour from 0 = red through to 127 = blue

    uint8_t red   = 0; // Red is the top 5 bits of a 16 bit colour value
    uint8_t green = 0;// Green is the middle 6 bits
    uint8_t blue  = 0; // Blue is the bottom 5 bits

    uint8_t sector = value >> 5;
    uint8_t amplit = value & 0x1F;

    switch (sector) {
    case 0:
        red   = 0x1F;
        green = amplit;
        blue  = 0;
        break;
    case 1:
        red   = 0x1F - amplit;
        green = 0x1F;
        blue  = 0;
        break;
    case 2:
        red   = 0;
        green = 0x1F;
        blue  = amplit;
        break;
    case 3:
        red   = 0;
        green = 0x1F - amplit;
        blue  = 0x1F;
        break;
    }

    return red << 11 | green << 6 | blue;
}

#else

#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The current arduino version of TFT_eSPI does not support arduino 3.0, please change the version to below 3.0");
    delay(1000);
}

#endif