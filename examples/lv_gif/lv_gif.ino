/**
 * @file      lv_gif.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 * @note      Place the gif file in the data directory into the SD card
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

#if !LV_USE_GIF
#error "lvgl gif decoder library is not enable!"
#endif

#define GIF_FILENAME    "/Pikachu.gif"

LV_IMG_DECLARE(Pikachu);

LilyGo_RGBPanel panel;

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels) {
                listDir(fs, file.path(), levels - 1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
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
    // Initialize SD Card
    if (!panel.installSD()) {
        Serial.println("Can't install SD Card!");
    }

    // Call lvgl initialization
    beginLvglHelper(panel);

    listDir(SD_MMC, "/", 0);


    lv_obj_t *img;
    img = lv_gif_create(lv_scr_act());
    lv_gif_set_src(img, &Pikachu);
    lv_obj_center(img);

    /* Assuming a File system is attached to letter 'A'
     E.g. A:/example.gif , A:example.png A:example.bin
    * or use  lvgl_helper_get_fs_filename function
    */

    /*
     if (!SD_MMC.exists(GIF_FILENAME)) {
         Serial.println("File not find image..."); return ;
     }
     const char *path = lvgl_helper_get_fs_filename(GIF_FILENAME);
     Serial.printf("lvgl_helper_get_fs_filename %s\n", path);

     img = lv_gif_create(lv_scr_act());
     lv_gif_set_src(img, path);
     lv_obj_center(img);
     */

    panel.setBrightness(16);
}


void loop()
{
    lv_timer_handler();
    // delay(2);
}

















