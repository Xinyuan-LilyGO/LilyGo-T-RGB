/**
 * @file      lv_images.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-22
 * @note      Place the images file in the data directory into the SD card
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

#if !LV_USE_PNG || !LV_USE_BMP || !LV_USE_SJPG
#error "lvgl png , bmp , sjpg decoder library is not enable!"
#endif

LilyGo_RGBPanel panel;

const char *filename[] = {
    "5.jpg", "6.jpg", "7.jpg",
    "8.jpg", "9.jpg", "10.jpg",
    "11.jpg",
};


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


    // Initialize T-RGB, if the initialization fails, false will be returned.
    if (!panel.begin()) {
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

    lv_obj_t *img =  lv_img_create(lv_scr_act());
    lv_obj_center(img);

    lv_timer_create([](lv_timer_t *t) {
        static int i = 0;

        if (!SD_MMC.exists(String("/image_") + filename[i])) {
            Serial.println("File not find image..."); return ;
        }

        // String path = LV_FS_POSIX_LETTER + String(":/image_") + filename[i];
        String path = lvgl_helper_get_fs_filename(String("/image_") + filename[i]);

        Serial.print("open : ");
        Serial.println(path);

        lv_img_set_src((lv_obj_t *)t->user_data, path.c_str());
        i++;
        i %= sizeof(filename) / sizeof(filename[0]);
    }, 5000, img);

    panel.setBrightness(16);
}


void loop()
{
    lv_timer_handler();
    delay(2);
}

















