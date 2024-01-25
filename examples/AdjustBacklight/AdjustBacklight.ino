/**
 * @file      AdjustBacklight.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-23
 *
 */
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

LilyGo_RGBPanel panel;
static lv_obj_t *label;
static lv_obj_t *label_level;

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

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

    lv_obj_t *arc = lv_arc_create(lv_scr_act());
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

    label_level = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(label_level, lv_color_white(), 0);
    lv_label_set_text(label_level, "16");
    lv_obj_set_style_text_font(label_level, &lv_font_montserrat_48, 0);
    lv_obj_center(label_level);

    label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "Level");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_align_to(label, label_level, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);


    lv_task_handler();

    // Set backlight level, range 0 ~ 16
    panel.setBrightness(16);

    lv_obj_add_event_cb(arc, [](lv_event_t *e) {
        lv_obj_t *obj =  lv_event_get_target(e);
        int16_t val =  lv_arc_get_value(obj);

        // Set backlight level, range 0 ~ 16
        panel.setBrightness(val);

        lv_label_set_text_fmt(label_level, "%d", val);

    }, LV_EVENT_VALUE_CHANGED, NULL);

}

void loop()
{
    lv_task_handler();
    delay(5);
}

