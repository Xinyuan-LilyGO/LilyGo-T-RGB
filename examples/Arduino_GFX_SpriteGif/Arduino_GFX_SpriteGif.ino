/*******************************************************************************
 * Sprite GIF Demo
 * This is a sprite demo using a static GIF as a master image
 * Image Source:
 * https://www.freepik.com/free-vector/urban-life-drawing_727890.htm#query=city%20road
 * https://giphy.com/gifs/car-carro-nQaMsylXcTIRNorQLJ
 ******************************************************************************/
//  * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below
//  * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below
//  * @note      2.1-inch and 2.8-inch have different initialization sequences, please turn on the correct option below

//! Place the gif file in the data directory into the SD card
#define GIF_FILENAME "/city17_240.gif"


#define GFX_DEV_DEVICE LILYGO_T_RGB
#include <Arduino_GFX_Library.h>
#include <Wire.h>

extern  const uint8_t st7701_type9_init_operations[297];

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
Arduino_RGB_Display     *gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST */, st7701_type4_init_operations, sizeof(st7701_type4_init_operations));

// ! T-RGB 2.8 inches Uncomment below
// Arduino_RGB_Display     *gfx = new Arduino_RGB_Display(
//     480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
//     bus, GFX_NOT_DEFINED /* RST */, st7701_type9_init_operations, sizeof(st7701_type9_init_operations));


Arduino_Canvas_Indexed *canvasGfx = new Arduino_Canvas_Indexed(480 /* width */, 480 /* height */, gfx);
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/
#include <SD_MMC.h>
#include "GifClass.h"

static GifClass gifClass;

uint8_t *spriteMaster;
bool spriteInitiated = false;

#include "IndexedSprite.h"
IndexedSprite *background;
IndexedSprite *road;
IndexedSprite *cars;
IndexedSprite *birds;
IndexedSprite *sun;
IndexedSprite *clouds;
IndexedSprite *mpv;

int frame = 0;
int fpsSnapShot = 0;
unsigned long nextSnap = 0;

void setup()
{
    Serial.begin(115200);
    // Serial.setDebugOutput(true);
    // while(!Serial);
    Serial.println("Arduino_GFX GIF Sprite example");

    Wire.begin(8 /* SDA */, 48 /* SCL */, 400000 /* speed */);

    // Init Display
    if (!canvasGfx->begin()) {
        Serial.println("canvasGfx->begin() failed!");
    }
    canvasGfx->fillScreen(BLACK);
    canvasGfx->flush();
    canvasGfx->setDirectUseColorIndex(true);

    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);

    //setup sdcard
    SD_MMC.setPins(39, 40, 38);
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println(F("ERROR: File System Mount Failed!"));
        gfx->println(F("ERROR: File System Mount Failed!"));
        exit(0);
    }

    uint8_t cardType = SD_MMC.cardType();
    if (cardType != CARD_NONE) {
        Serial.print("SD Card Type: ");
        if (cardType == CARD_MMC)
            Serial.println("MMC");
        else if (cardType == CARD_SD)
            Serial.println("SDSC");
        else if (cardType == CARD_SDHC)
            Serial.println("SDHC");
        else
            Serial.println("UNKNOWN");
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
    }


    File gifFile = SD_MMC.open(GIF_FILENAME, FILE_READ);
    if (!gifFile || gifFile.isDirectory()) {
        Serial.println(F("ERROR: open gifFile Failed!"));
        gfx->println(F("ERROR: open gifFile Failed!"));
    } else {
        // read GIF file header
        gd_GIF *gif = gifClass.gd_open_gif(&gifFile);
        if (!gif) {
            Serial.println(F("gd_open_gif() failed!"));
        } else {
            spriteMaster = (uint8_t *)malloc(gif->width * gif->height / 2);
            if (!spriteMaster) {
                Serial.println(F("spriteMaster malloc failed!"));
            } else {
                int32_t res = gifClass.gd_get_frame(gif, spriteMaster);

                if (res > 0) {
                    // inital palette
                    uint16_t *palette = canvasGfx->getColorIndex();
                    memcpy(palette, gif->palette->colors, gif->palette->len * 2);

                    //IndexedSprite(x, y, *bitmap, *palette, w, h, x_skip, loop, frames, speed_divider, chroma_key)
                    background = new IndexedSprite(0, 0, spriteMaster, palette, 405, 180, 0, true, 1, 3);
                    road = new IndexedSprite(0, 180, spriteMaster + (180 * 405), palette, 405, 60, 0, true, 1, 1);
                    cars = new IndexedSprite(0, 182, spriteMaster + (240 * 405), palette, 405, 11, 0, true, 1, 1, gif->gce.tindex);
                    birds = new IndexedSprite(0, 80, spriteMaster + (251 * 405), palette, 51, 32, (405 - 51), false, 4, 4, gif->gce.tindex);
                    sun = new IndexedSprite(16, 16, spriteMaster + (251 * 405) + 210, palette, 30, 30, (405 - 30), false, 1, 0, gif->gce.tindex);
                    clouds = new IndexedSprite(0, 2, spriteMaster + (283 * 405), palette, 405, 94, 0, true, 1, 2, gif->gce.tindex);
                    mpv = new IndexedSprite((canvasGfx->width() - 70) / 2, 182, spriteMaster + (377 * 405), palette, 50, 30, (405 - 50), false, 8, 2, gif->gce.tindex);

                    spriteInitiated = true;
                }

                gifClass.gd_close_gif(gif);
            }
        }
    }

    canvasGfx->setTextColor(0xfd, 0x00);
    canvasGfx->setTextSize(1);
}

bool otherFrame = false;
void testingLoop(void)
{
    if (spriteInitiated) {
        background->h_scroll(-1);
        background->draw(canvasGfx);

        road->h_scroll(-3);
        road->draw(canvasGfx);

        cars->h_scroll(-6);
        cars->draw(canvasGfx);

        birds->h_scroll(1, 480);
        birds->next_frame();
        birds->draw(canvasGfx);

        sun->draw(canvasGfx);

        clouds->h_scroll(1);
        clouds->draw(canvasGfx);

        mpv->next_frame();
        mpv->draw(canvasGfx);
    }
}

void loop()
{
    testingLoop();

    canvasGfx->setCursor(8, 8);
    canvasGfx->print(fpsSnapShot);

    canvasGfx->flush();

    // calculate FPS
    frame++;
    if (millis() > nextSnap) {
        fpsSnapShot = frame;
        frame = 0;
        nextSnap = millis() + 1000;
    }
}

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