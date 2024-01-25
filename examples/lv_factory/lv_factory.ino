/**
 * @file      lv_factory.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-24
 *
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <WiFi.h>
#include "rootCa.h"
#include "zones.h"

#ifndef WIFI_SSID
#define WIFI_SSID             "Your WiFi SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD         "Your WiFi PASSWORD"
#endif

#define NTP_SERVER1           "pool.ntp.org"
#define NTP_SERVER2           "time.nist.gov"
#define GMT_OFFSET_SEC        0
#define DAY_LIGHT_OFFSET_SEC  0
#define GET_TIMEZONE_API      "https://ipapi.co/timezone/"
#define DEFAULT_TIMEZONE      "CST-8"         //When the time zone cannot be obtained, the default time zone is used

#define WIFI_MSG_ID             0x1001

LV_IMG_DECLARE(image_1_180x180);
LV_IMG_DECLARE(icon_cpu);
LV_IMG_DECLARE(icon_flash);
LV_IMG_DECLARE(icon_ram);
LV_IMG_DECLARE(icon_micro_sd);
LV_IMG_DECLARE(image_1_480x480);
LV_IMG_DECLARE(image_2_480x480);


//Arduino IDE cannot select the partition table as default_16MB.csv. Delete the image to reduce the size.
// In platformio it is not affected
// LV_IMG_DECLARE(image_3_480x480);     

const void *images_bg[] = {
    &image_1_480x480,
    &image_2_480x480,
    // &image_3_480x480,    
};

LilyGo_RGBPanel panel;
WiFiClientSecure client ;
HTTPClient https;

static TaskHandle_t  vUpdateDateTimeTaskHandler = NULL;
static String httpBody;
static SemaphoreHandle_t xWiFiLock = NULL;

static void lv_battery_gui(lv_obj_t *parent);
static void lv_backlight_gui(lv_obj_t *parent);
static void lv_weather_gui(lv_obj_t *parent);
static void lv_info_gui(lv_obj_t *parent);
static void lv_factory_gui_init();
static void lv_clock_gui(lv_obj_t *paran);
static void WiFiEvent(WiFiEvent_t event);
static void datetimeSyncTask(void *ptr);
static void lv_wifi_gui(lv_obj_t *parent);

void setup()
{
    Serial.begin(115200);

    xWiFiLock =  xSemaphoreCreateBinary();
    xSemaphoreGive( xWiFiLock );

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

    // Factory page drawing
    lv_factory_gui_init();

    // Set backlight level, range 0 ~ 16
    panel.setBrightness(16);

    // Register WiFi event
    WiFi.onEvent(WiFiEvent);

    WiFi.mode(WIFI_STA);
    // Just for factory testing.
    Serial.print("Use default WiFi SSID & PASSWORD!!");
    Serial.print("SSID:"); Serial.println(WIFI_SSID);
    Serial.print("PASSWORD:"); Serial.println(WIFI_PASSWORD);
    if (String(WIFI_SSID) == "Your WiFi SSID" || String(WIFI_PASSWORD) == "Your WiFi PASSWORD" ) {
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
        Serial.println("[Error] : WiFi ssid and password are not configured correctly");
    }
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);


}


void loop()
{
    lv_timer_handler();
    delay(2);
}

static void tileview_change_cb(lv_event_t *e)
{
    lv_obj_t *tileview = lv_event_get_target(e);
    lv_obj_t *act = lv_tileview_get_tile_act(tileview);
    lv_event_code_t c = lv_event_get_code(e);
    Serial.print("Code : ");
    Serial.println(c);
    if (act) {
        uint8_t pageId = lv_obj_get_index(act);
        uint32_t count =  lv_obj_get_child_cnt(tileview);
        Serial.print(" Count:");
        Serial.print(count);
        Serial.print(" pageId:");
        Serial.println(pageId);
    }
}

static void lv_factory_gui_init()
{
    static lv_style_t bgStyle;
    lv_style_init(&bgStyle);
    lv_style_set_bg_color(&bgStyle, lv_color_black());

    lv_obj_t *tileview = lv_tileview_create(lv_scr_act());
    lv_obj_add_style(tileview, &bgStyle, LV_PART_MAIN);
    lv_obj_set_size(tileview, LV_PCT(100), LV_PCT(100));
    lv_obj_add_event_cb(tileview, tileview_change_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(tileview, tileview_change_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *t1 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t2 = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t3 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t4 = lv_tileview_add_tile(tileview, 3, 0,  LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t5 = lv_tileview_add_tile(tileview, 4, 0,  LV_DIR_HOR | LV_DIR_BOTTOM);
    lv_obj_t *t6 = lv_tileview_add_tile(tileview, 5, 0,  LV_DIR_HOR | LV_DIR_BOTTOM);

    lv_clock_gui(t1);
    lv_weather_gui(t2);
    lv_battery_gui(t3);
    lv_backlight_gui(t4);
    lv_wifi_gui(t5);
    lv_info_gui(t6);
}



static void lv_battery_gui(lv_obj_t *parent)
{

    static lv_obj_t *label_voltage;
    static lv_obj_t *label;
    static lv_obj_t *meter;
    static lv_meter_indicator_t *indic2 ;
    static lv_meter_indicator_t *indic1 ;

    meter = lv_meter_create(parent);

    /*Remove the background and the circle from the middle*/
    lv_obj_remove_style(meter, NULL, LV_PART_MAIN);
    lv_obj_remove_style(meter, NULL, LV_PART_INDICATOR);

    lv_obj_set_size(meter, LV_PCT(100), LV_PCT(100));
    lv_obj_center(meter);

    /*Add a scale first with no ticks.*/
    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 0, 0, 0, lv_color_black());
    lv_meter_set_scale_range(meter, scale, 0, 100, 280, 130);

    /*Add a three arc indicator*/
    lv_coord_t indic_w = 100;

    indic1 = lv_meter_add_arc(meter, scale, indic_w, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_meter_set_indicator_start_value(meter, indic1, 0);
    lv_meter_set_indicator_end_value(meter, indic1, 100);

    indic2 = lv_meter_add_arc(meter, scale, indic_w, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_meter_set_indicator_start_value(meter, indic2, 0);
    lv_meter_set_indicator_end_value(meter, indic2, 90);


    label_voltage = lv_label_create(parent);
    lv_obj_set_style_text_color(label_voltage, lv_color_white(), 0);
    lv_label_set_text(label_voltage, "0");
    lv_obj_set_style_text_font(label_voltage, &lv_font_montserrat_48, 0);
    lv_obj_center(label_voltage);

    label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "Volts");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_align_to(label, label_voltage, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_timer_create([](lv_timer_t *t) {

        uint16_t battery_voltage = panel.getBattVoltage();

        int percentage = 0;

        if (battery_voltage > 4200) {
            percentage = 100;
            battery_voltage = 4200;
        } else {
            percentage = (uint8_t)((((battery_voltage / 1000.0) - 3.0) / (4.2 - 3.0)) * 100);
        }

        lv_label_set_text_fmt(label_voltage, "%.2f", battery_voltage / 1000.0);

        lv_meter_set_indicator_end_value(meter, indic2, percentage);

    }, 1000, NULL);
}



static void lv_wifi_gui(lv_obj_t *parent)
{

    static lv_obj_t *label_rssi;
    static lv_obj_t *label;
    static lv_obj_t *meter;
    static lv_meter_indicator_t *indic2 ;
    static lv_meter_indicator_t *indic1 ;

    meter = lv_meter_create(parent);

    /*Remove the background and the circle from the middle*/
    lv_obj_remove_style(meter, NULL, LV_PART_MAIN);
    lv_obj_remove_style(meter, NULL, LV_PART_INDICATOR);

    lv_obj_set_size(meter, LV_PCT(100), LV_PCT(100));
    lv_obj_center(meter);

    /*Add a scale first with no ticks.*/
    lv_meter_scale_t *scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 0, 0, 0, lv_color_black());
    lv_meter_set_scale_range(meter, scale, -100, 10, 280, 130);

    /*Add a three arc indicator*/
    lv_coord_t indic_w = 100;

    indic1 = lv_meter_add_arc(meter, scale, indic_w, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_meter_set_indicator_start_value(meter, indic1, 0);
    lv_meter_set_indicator_end_value(meter, indic1, 100);

    indic2 = lv_meter_add_arc(meter, scale, indic_w, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic2, 0);
    lv_meter_set_indicator_end_value(meter, indic2, 90);


    label_rssi = lv_label_create(parent);
    lv_obj_set_style_text_color(label_rssi, lv_color_white(), 0);
    lv_label_set_text(label_rssi, "0");
    lv_obj_set_style_text_font(label_rssi, &lv_font_montserrat_48, 0);
    lv_obj_center(label_rssi);

    label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "RSSI");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_align_to(label, label_rssi, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_timer_create([](lv_timer_t *t) {

        if (!WiFi.isConnected()) {
            return;
        }
        int32_t rssi = WiFi.RSSI();
        lv_label_set_text_fmt(label_rssi, "%d",  rssi);

        lv_meter_set_indicator_end_value(meter, indic2, rssi);

    }, 1000, NULL);
}

static void lv_backlight_gui(lv_obj_t *parent)
{
    static lv_obj_t *label;
    static lv_obj_t *label_level;

    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_style_arc_width(arc, 100, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 100, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 100, LV_PART_KNOB);
    lv_obj_set_style_arc_rounded(arc, 0, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc, 0, LV_PART_INDICATOR);

    lv_obj_set_style_arc_color(arc, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_INDICATOR);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_set_size(arc, LV_PCT(100), LV_PCT(100));
    lv_arc_set_angles(arc, 0, 280);
    lv_arc_set_rotation(arc, 130);
    lv_arc_set_bg_angles(arc, 0, 280);
    lv_arc_set_range(arc, 1, 16);
    lv_arc_set_value(arc, 16);
    lv_obj_center(arc);

    lv_obj_add_flag(arc,
                    LV_OBJ_FLAG_SCROLLABLE |
                    LV_OBJ_FLAG_SCROLL_CHAIN_HOR |
                    LV_OBJ_FLAG_EVENT_BUBBLE |
                    LV_OBJ_FLAG_GESTURE_BUBBLE |
                    LV_OBJ_FLAG_ADV_HITTEST
                   );

    label_level = lv_label_create(parent);
    lv_obj_set_style_text_color(label_level, lv_color_white(), 0);
    lv_label_set_text(label_level, "16");
    lv_obj_set_style_text_font(label_level, &lv_font_montserrat_48, 0);
    lv_obj_center(label_level);


    label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "Level");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_align_to(label, label_level, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_obj_add_event_cb(arc, [](lv_event_t *e) {

        lv_obj_t *obj =  lv_event_get_target(e);
        int16_t val =  lv_arc_get_value(obj);

        // Set backlight level, range 0 ~ 16
        panel.setBrightness(val);

        lv_label_set_text_fmt(label_level, "%d", val);

    }, LV_EVENT_VALUE_CHANGED, NULL);
}



static void lv_img_click_event(lv_event_t *e)
{
    static uint8_t i = 1;
    lv_obj_t *parent = (lv_obj_t *)lv_event_get_user_data(e);
    lv_obj_set_style_bg_img_src(parent, images_bg[i], 0);
    i++;
    i %= sizeof(images_bg) / sizeof(images_bg[0]);
}

static void lv_weather_gui(lv_obj_t *parent)
{
    lv_obj_set_style_bg_img_src(parent, &image_1_480x480, 0);

    lv_obj_t *img =  lv_img_create(parent);
    lv_img_set_src(img, &image_1_180x180);
    lv_obj_align(img, LV_ALIGN_CENTER, 0, -20);
    lv_obj_add_event_cb(img, lv_img_click_event, LV_EVENT_CLICKED, parent);
    lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "24°C");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_align_to(label, img, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

}

static void lv_info_gui(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_bg_color(cont, lv_color_black(), 0);
    lv_obj_set_size(cont, lv_disp_get_physical_hor_res(NULL), lv_disp_get_ver_res(NULL) );
    lv_obj_remove_style(cont, 0, LV_PART_SCROLLBAR);
    lv_obj_center(cont);
    lv_obj_set_style_border_width(cont, 0, 0);

    static lv_style_t font_style;
    lv_style_init(&font_style);
    lv_style_set_text_color(&font_style, lv_color_white());
    lv_style_set_text_font(&font_style, &lv_font_montserrat_18);

    // CPU
    lv_obj_t *img_cpu = lv_img_create(cont);
    lv_img_set_src(img_cpu, &icon_cpu);
    lv_obj_align(img_cpu, LV_ALIGN_LEFT_MID, 55, -50);

    lv_obj_t *label = lv_label_create(cont);
    lv_obj_add_style(label, &font_style, 0);
    lv_label_set_text(label, ESP.getChipModel());
    lv_obj_align_to(label, img_cpu, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    //Flash
    lv_obj_t *img_flash = lv_img_create(cont);
    lv_img_set_src(img_flash, &icon_flash);
    lv_obj_align_to(img_flash, img_cpu, LV_ALIGN_OUT_RIGHT_MID, 40, 0);

    label = lv_label_create(cont);
    lv_obj_add_style(label, &font_style, 0);
    lv_label_set_text_fmt(label, "%uMB", ESP.getFlashChipSize() / 1024 / 1024);
    lv_obj_align_to(label, img_flash, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);


    //PSRAM
    lv_obj_t *img_ram = lv_img_create(cont);
    lv_img_set_src(img_ram, &icon_ram);
    lv_obj_align_to(img_ram, img_flash, LV_ALIGN_OUT_RIGHT_MID, 40, 0);

    float ram_size = abs(ESP.getPsramSize() / 1024.0 / 1024.0);
    label = lv_label_create(cont);
    lv_obj_add_style(label, &font_style, 0);
    lv_label_set_text_fmt(label, "%.1fMB", ram_size);
    lv_obj_align_to(label, img_ram, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    lv_obj_t *img_sd = lv_img_create(cont);
    lv_img_set_src(img_sd, &icon_micro_sd);
    lv_obj_align_to(img_sd, img_ram, LV_ALIGN_OUT_RIGHT_MID, 40, 0);

    label = lv_label_create(cont);
    lv_obj_add_style(label, &font_style, 0);
    if (SD_MMC.cardType() != CARD_NONE) {
        lv_label_set_text_fmt(label, "%.2fG", SD_MMC.cardSize() / 1024 / 1024 / 1024.0);
    } else {
        lv_label_set_text(label, "N/A");
    }
    lv_obj_align_to(label, img_sd, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

    lv_obj_t *wifi_name = lv_label_create(cont);
    lv_label_set_text(wifi_name, "N/A");
    lv_obj_set_style_text_font(wifi_name, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(wifi_name, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(wifi_name, LV_ALIGN_CENTER, 0, 55);


    lv_obj_t *wifi_address = lv_label_create(cont);
    lv_label_set_text(wifi_address, "IP:NONE" );
    lv_obj_set_style_text_font(wifi_address, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(wifi_address, lv_color_white(), LV_PART_MAIN);
    lv_obj_align_to(wifi_address, wifi_name, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    lv_msg_subsribe_obj(WIFI_MSG_ID, wifi_address, NULL);

    // Added got ip address message cb
    lv_obj_add_event_cb( wifi_address, [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        lv_obj_t *name = (lv_obj_t *)lv_event_get_user_data(e);

        Serial.print("-->>IP:"); Serial.println(WiFi.localIP());
        lv_label_set_text_fmt(name, "SSID:%s", WiFi.SSID().c_str());
        lv_obj_align(name, LV_ALIGN_CENTER, 0, 55);
        lv_label_set_text_fmt(label, "IP:%s",  WiFi.isConnected() ? (WiFi.localIP().toString().c_str()) : ("NONE") );
        lv_obj_align_to(label, name, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    }, LV_EVENT_MSG_RECEIVED, wifi_name);

}


static lv_obj_t *clock_meter;
static lv_meter_indicator_t *indic_min ;
static lv_meter_indicator_t *indic_hour;

static void lv_clock_gui(lv_obj_t *paran)
{
    clock_meter = lv_meter_create(paran);
    lv_obj_set_size(clock_meter, LV_PCT(100), LV_PCT(100));
    lv_obj_center(clock_meter);

    /*Create a scale for the minutes*/
    /*61 ticks in a 360 degrees range (the last and the first line overlaps)*/
    lv_meter_scale_t *scale_min = lv_meter_add_scale(clock_meter);
    lv_meter_set_scale_ticks(clock_meter, scale_min, 61, 1, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_range(clock_meter, scale_min, 0, 60, 360, 270);

    /*Create another scale for the hours. It's only visual and contains only major ticks*/
    lv_meter_scale_t *scale_hour = lv_meter_add_scale(clock_meter);
    lv_meter_set_scale_ticks(clock_meter, scale_hour, 12, 0, 0, lv_palette_main(LV_PALETTE_GREY)); /*12 ticks*/
    lv_meter_set_scale_major_ticks(clock_meter, scale_hour, 1, 2, 20, lv_color_black(), 10);       /*Every tick is major*/
    lv_meter_set_scale_range(clock_meter, scale_hour, 1, 12, 330, 300);                            /*[1..12] values in an almost full circle*/

    LV_IMG_DECLARE(img_hand)

    /*Add a the hands from images*/
    indic_min  = lv_meter_add_needle_img(clock_meter, scale_min, &img_hand, 5, 5);
    indic_hour = lv_meter_add_needle_img(clock_meter, scale_min, &img_hand, 5, 5);

    lv_timer_create([](lv_timer_t *t) {
        struct tm timeinfo;
        time_t now;
        time(&now);
        localtime_r(&now, &timeinfo);
        lv_meter_set_indicator_end_value(clock_meter, indic_min, timeinfo.tm_min);
        lv_meter_set_indicator_end_value(clock_meter, indic_hour, timeinfo.tm_hour);
    }, 1000, NULL);
}


static void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
    case ARDUINO_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        lv_msg_send(WIFI_MSG_ID, NULL);
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        configTime(GMT_OFFSET_SEC, DAY_LIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
        if (!vUpdateDateTimeTaskHandler) {
            xTaskCreate(datetimeSyncTask, "sync", 10 * 1024, NULL, 12, &vUpdateDateTimeTaskHandler);
        }
        lv_msg_send(WIFI_MSG_ID, NULL);
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        lv_msg_send(WIFI_MSG_ID, NULL);
        break;
    default: break;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
// DATETIME DATETIME DATETIME DATETIME
/////////////////////////////////////////////////////////////////////////////////////////
static void datetimeSyncTask(void *ptr)
{
    int httpCode;
    while (1) {
        delay(5000);

        if ( xSemaphoreTake( xWiFiLock, portMAX_DELAY ) == pdTRUE ) {

            // When the time zone cannot be obtained, please check the validity of the certificate
            client.setCACert(rootCACertificate);
            if (https.begin(client, GET_TIMEZONE_API)) {
                httpCode = https.GET();
                if (httpCode > 0) {
                    Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
                    if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
                        httpBody = https.getString();
                    }
                } else {
                    Serial.printf("[HTTPS] GET... failed, error: %s\n",
                                  https.errorToString(httpCode).c_str());
                    httpBody = "none";
                }
                https.end();
            }

            client.stop();

            for (uint32_t i = 0; i < sizeof(zones); i++) {
                if (httpBody == "none") {
                    Serial.println("Failed to obtain time zone, use default time zone");
                    // When the time zone cannot be obtained, the default time zone is used
                    httpBody = DEFAULT_TIMEZONE;
                    break;
                }
                if (httpBody == zones[i].name) {
                    httpBody = zones[i].zones;
                    break;
                }
            }
            Serial.println("timezone : " + httpBody);
            setenv("TZ", httpBody.c_str(), 1); // set time zone
            tzset();
            xSemaphoreGive( xWiFiLock );

            vUpdateDateTimeTaskHandler = NULL;
            // Just run once
            vTaskDelete(NULL);
        }
    }
}
