/**
 * @file      DeepSleep.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-03-18
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>


#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time T-RGB will go to sleep (in seconds) */


LilyGo_RGBPanel panel;
lv_obj_t *btn_label ;
bool gotoSleep = false;

static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;
        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
        Serial.printf("Button :%d\n", cnt);
        gotoSleep = true;
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

    // Call lvgl initialization
    beginLvglHelper(panel);


    lv_obj_t *label = lv_label_create(lv_scr_act());        /*Add a label the current screen*/
    lv_label_set_text(label, "Hello World");                 /*Set label text*/
    lv_obj_center(label);                                   /*Set center alignment*/


    lv_obj_t *btn = lv_btn_create(lv_scr_act());            /*Add a button the current screen*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);  /*Assign a callback to the button*/
    lv_obj_align_to(btn, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 0); /*Set the label to it and align it in the center below the label*/

    btn_label = lv_label_create(btn);           /*Add a label to the button*/
    lv_label_set_text(btn_label, "Button");               /*Set the labels text*/
    lv_obj_center(btn_label);

    // Turn on the backlight and set it to the highest value, ranging from 0 to 16
    panel.setBrightness(16);
}



void loop()
{

    if (gotoSleep) {

        Serial.println("Sleep!");

        // Show sleep text
        lv_label_set_text(btn_label, "Sleep start ..");
        lv_timer_handler();
        delay(2000);

        // panel.enableButtonWakeup();          // Use BUTTON 0 (BOOT) button to wake up
        panel.enableTouchWakeup();      // Use touch screen to wake up
        // panel.enableTimerWakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);     // Use a timer to set a specified time interval to wake up

        // Call sleep. This method will not return any value and go to sleep directly.
        panel.sleep();
    }

    // lvgl task processing should be placed in the loop function
    lv_timer_handler();
    delay(2);
}

















