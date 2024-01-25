/**
 * @file      Arduino_GFX_HelloWorld.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-23
 * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below
 * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below
 * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below
 */
#define GFX_DEV_DEVICE LILYGO_T_RGB
#include <Arduino_GFX_Library.h>
#include <Wire.h>
extern const uint8_t st7701_type9_init_operations[297];

#define GFX_BL 46
Arduino_DataBus *bus = new Arduino_XL9535SWSPI(8 /* SDA */, 48 /* SCL */, 2 /* XL PWD */, 3 /* XL CS */, 5 /* XL SCK */, 4 /* XL MOSI */);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    45 /* DE */, 41 /* VSYNC */, 47 /* HSYNC */, 42 /* PCLK */,
    21 /* R0 */, 18 /* R1 */, 17 /* R2 */, 16 /* R3 */, 15 /* R4 */,
    14 /* G0 */, 13 /* G1 */, 12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */,
    7 /* B0 */, 6 /* B1 */, 5 /* B2 */, 3 /* B3 */, 2 /* B4 */,
    1 /* hsync_polarity */, 50 /* hsync_front_porch */, 1 /* hsync_pulse_width */, 30 /* hsync_back_porch */,
    1 /* vsync_polarity */, 20 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 30 /* vsync_back_porch */,
    1 /* pclk_active_neg */);

// ! T-RGB 2.1 inches 
// Arduino_RGB_Display     *gfx = new Arduino_RGB_Display(
//     480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
//     bus, GFX_NOT_DEFINED /* RST */, st7701_type4_init_operations, sizeof(st7701_type4_init_operations));

// ! T-RGB 2.8 inches Uncomment below
Arduino_RGB_Display     *gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST */, st7701_type9_init_operations, sizeof(st7701_type9_init_operations));


void setup(void)
{
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
    // while(!Serial);
    Serial.println("Arduino_GFX Hello World example");

    Wire.begin(8 /* SDA */, 48 /* SCL */, 400000 /* speed */);

    // Init Display
    if (!gfx->begin()) {
        Serial.println("gfx->begin() failed!");
    }
    gfx->fillScreen(BLACK);

    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);

    gfx->setCursor(10, 10);
    gfx->setTextColor(RED);
    gfx->println("Hello World!");

    delay(5000); // 5 seconds
}

void loop()
{
    gfx->setCursor(random(gfx->width()), random(gfx->height()));
    gfx->setTextColor(random(0xffff), random(0xffff));
    gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
    gfx->println("Hello World!");

    delay(1000); // 1 second
}


// 2.8-inch initialization sequence
const uint8_t st7701_type9_init_operations[297] = {
    BEGIN_WRITE,

    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x13,

    WRITE_C8_D8, 0xEF, 0x08,

    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x10,

    WRITE_C8_D16, 0xC0, 0x3B, 0x00,
    WRITE_C8_D16, 0xC1, 0x10, 0x0C,
    WRITE_C8_D16, 0xC2, 0x07, 0x0A,
    WRITE_C8_D8, 0xC7, 0x00,
    WRITE_C8_D8, 0xC7, 0x10,
    WRITE_C8_D8, 0xCD, 0x08,

    WRITE_COMMAND_8, 0xB0, // Positive Voltage Gamma Control
    WRITE_BYTES, 16,
    0x05, 0x12, 0x98, 0x0e, 0x0F,
    0x07, 0x07, 0x09, 0x09, 0x23,
    0x05, 0x52, 0x0F, 0x67, 0x2C,
    0x11,



    WRITE_COMMAND_8, 0xB1, // Negative Voltage Gamma Control
    WRITE_BYTES, 16,
    0x0B, 0x11, 0x97, 0x0C, 0x12,
    0x06, 0x06, 0x08, 0x08, 0x22,
    0x03, 0x51, 0x11, 0x66, 0x2B,
    0x0F,

    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x11,

    WRITE_C8_D8, 0xB0, 0x5D,
    WRITE_C8_D8, 0xB1, 0x2D,
    WRITE_C8_D8, 0xB2, 0x81,
    WRITE_C8_D8, 0xB3, 0x80,
    WRITE_C8_D8, 0xB5, 0x4E,
    WRITE_C8_D8, 0xB7, 0x85,
    WRITE_C8_D8, 0xB8, 0x20,
    WRITE_C8_D8, 0xC1, 0x78,
    WRITE_C8_D8, 0xC2, 0x78,

    WRITE_C8_D8, 0xD0, 0x88,

    WRITE_COMMAND_8, 0xE0,
    WRITE_BYTES, 4,
    0x00, 0x00, 0x02,

    WRITE_COMMAND_8, 0xE1,
    WRITE_BYTES, 16,
    0x06, 0x30, 0x08, 0x30, 0x05,
    0x30, 0x07, 0x30, 0x00, 0x33,
    0x33,

    WRITE_COMMAND_8, 0xE2,
    WRITE_BYTES, 16,
    0x11, 0x11, 0x33, 0x33, 0xf4,
    0x00, 0x00, 0x00, 0xf4, 0x00,
    0x00, 0x00,

    WRITE_COMMAND_8, 0xE3,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,

    WRITE_C8_D16, 0xE4, 0x44, 0x44,

    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 16,
    0x0d, 0xf5, 0x30, 0xf0, 0x0f,
    0xf7, 0x30, 0xf0, 0x09, 0xf1,
    0x30, 0xf0, 0x0b, 0xf3, 0x30, 0xf0,

    WRITE_COMMAND_8, 0xE6,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,

    WRITE_C8_D16, 0xE7, 0x44, 0x44,

    WRITE_COMMAND_8, 0xE8,
    WRITE_BYTES, 16,
    0x0c, 0xf4, 0x30, 0xf0,
    0x0e, 0xf6, 0x30, 0xf0,
    0x08, 0xf0, 0x30, 0xf0,
    0x0a, 0xf2, 0x30, 0xf0,

    WRITE_C8_D8, 0xe9, 0x36,

    WRITE_COMMAND_8, 0xEB,
    WRITE_BYTES, 7,
    0x00, 0x01, 0xe4, 0xe4,
    0x44, 0x88, 0x40,

    WRITE_COMMAND_8, 0xED,
    WRITE_BYTES, 16,
    0xff, 0x10, 0xaf, 0x76,
    0x54, 0x2b, 0xcf, 0xff,
    0xff, 0xfc, 0xb2, 0x45,
    0x67, 0xfa, 0x01, 0xff,

    WRITE_COMMAND_8, 0xEF,
    WRITE_BYTES, 6,
    0x08, 0x08, 0x08, 0x45,
    0x3f, 0x54,

    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x00,

    WRITE_COMMAND_8, 0x11, // Sleep Out
    END_WRITE,

    DELAY, 150,

    BEGIN_WRITE,
    WRITE_C8_D8, 0x3A, 0x66,
    WRITE_C8_D8, 0x36, 0x08,
    WRITE_C8_D8, 0x35, 0x00,
    WRITE_COMMAND_8, 0x29, // Display On
    END_WRITE,
    DELAY, 20
};