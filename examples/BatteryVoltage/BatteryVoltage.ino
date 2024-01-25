/**
 * @file      BatteryVoltage.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-23
 *
 */
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>

LilyGo_RGBPanel panel;
static lv_obj_t *label_voltage;
static lv_obj_t *label;
static lv_obj_t *meter;
static lv_meter_indicator_t *indic2 ;
static lv_meter_indicator_t *indic1 ;


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

    meter = lv_meter_create(lv_scr_act());

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


    label_voltage = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(label_voltage, lv_color_white(), 0);
    lv_label_set_text(label_voltage, "0");
    lv_obj_set_style_text_font(label_voltage, &lv_font_montserrat_48, 0);
    lv_obj_center(label_voltage);

    label = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_label_set_text(label, "Volts");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_align_to(label, label_voltage, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_task_handler();

    panel.setBrightness(16);

    lv_timer_create([](lv_timer_t *t) {

        uint16_t battery_voltage = panel.getBattVoltage();

        int percentage = 0;

        if (battery_voltage > 4200) {
            percentage = 100;
            battery_voltage = 4200;
        } else {
            percentage = (uint8_t)((((battery_voltage / 1000.0) - 3.0) / (4.2 - 3.0)) * 100);
        }

        Serial.printf("V:%.2f  %d\n", battery_voltage / 1000.0, percentage);

        lv_label_set_text_fmt(label_voltage, "%.2f", battery_voltage / 1000.0);

        lv_meter_set_indicator_end_value(meter, indic2, percentage);

    }, 1000, NULL);
}

void loop()
{
    lv_task_handler();
    delay(5);
}

