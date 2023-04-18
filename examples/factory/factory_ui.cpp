#include "factory_ui.h"
#include "Arduino.h"
#include "SD_MMC.h"
#include "lvgl.h"

extern void deep_sleep();

void ui_begin()
{
    String buf;
    lv_obj_t *cout = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cout, LV_PCT(100), LV_PCT(100));
    lv_obj_set_scroll_dir(cout, LV_DIR_NONE);

    lv_obj_t *chip_info = lv_label_create(cout);
    lv_obj_set_width(chip_info, LV_PCT(50));
    lv_obj_set_style_text_align(chip_info, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(chip_info, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_recolor(chip_info, true);

    buf = "Board info : T-RGB-V1.0";
    buf += "\r\nChip: ";
    buf += ESP.getChipModel();
    buf += "\r\nChipRevision: ";
    buf += ESP.getChipRevision();
    buf += "\r\nPsram size: ";
    buf += ESP.getPsramSize() / 1024;
    buf += "KB";
    buf += "\r\nFlash size: ";
    buf += ESP.getFlashChipSize() / 1024;
    buf += "KB";
    buf += "\r\nCPU frequency: ";
    buf += ESP.getCpuFreqMHz();
    buf += "MHz";
    buf += "\r\nUse ";
    buf +=  getTouchAddr();
    buf += " Touch Dev";


    if (SD_MMC.cardSize()) {
        buf += "\r\nSD Card Size: #00ff00 ";
        buf += int(SD_MMC.cardSize() / (1024 * 1024));
        buf += " MB#";
    } else {
        buf += "\r\nSD:#ff0000 Card Mount Failed#";
    }

    lv_label_set_text(chip_info, buf.c_str());

    lv_obj_t *bat_label = lv_label_create(cout);
    lv_obj_align_to(bat_label, chip_info, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_msg_subsribe_obj(MSG_BAT_VOLT_UPDATE, bat_label, NULL);
    lv_obj_add_event_cb(
        bat_label,
    [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        lv_msg_t *m = lv_event_get_msg(e);
        const float *v = (const float *)lv_msg_get_payload(m);
        lv_label_set_text_fmt(label, "bat volt : %.2f V\r\n", *v);
    },
    LV_EVENT_MSG_RECEIVED, NULL);

    lv_obj_t *tpoint_label = lv_label_create(cout);
    lv_obj_align_to(tpoint_label, bat_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_label_set_recolor(tpoint_label, true);
    lv_obj_add_event_cb(
        tpoint_label,
    [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        lv_msg_t *m = lv_event_get_msg(e);
        const touch_point_t *v = (const touch_point_t *)lv_msg_get_payload(m);
        if (v->x && v->y)
            lv_label_set_text_fmt(label, "touch #00ff00 x: %d , y: %d #\r\n", v->x, v->y);
        else
            lv_label_set_text_fmt(label, "touch #0000ff x: %d , y: %d #\r\n", v->x, v->y);
    },
    LV_EVENT_MSG_RECEIVED, NULL);
    lv_msg_subsribe_obj(MSG_TOUCH_UPDATE, tpoint_label, NULL);

    lv_obj_t *touch_status = lv_label_create(cout);
    lv_obj_align_to(touch_status, tpoint_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_label_set_recolor(touch_status, true);
    lv_obj_add_event_cb(
    touch_status,   [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        lv_msg_t *m = lv_event_get_msg(e);
        bool *v = ( bool *)lv_msg_get_payload(m);
        if (*v)
            lv_label_set_text_fmt(label, "touch #00ff00 Pressed\r\n");
        else
            lv_label_set_text_fmt(label, "touch #0000ff Release#\r\n");
    },
    LV_EVENT_MSG_RECEIVED, NULL);
    lv_msg_subsribe_obj(MSG_TOUCH_INT_UPDATE, touch_status, NULL);

    lv_obj_t *btn = lv_btn_create(cout);
    lv_obj_align_to(btn, touch_status, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_set_height(btn, 50);
    lv_obj_add_event_cb( btn, [](lv_event_t *e) {
        deep_sleep();
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_obj_center(btn_label);
    lv_label_set_text(btn_label, "Click on Me Enter Deep sleep");

    lv_obj_t *wifi_label = lv_label_create(cout);
    lv_obj_set_width(wifi_label, LV_PCT(60));
    lv_obj_set_height(wifi_label, LV_PCT(120));
    lv_obj_align_to(wifi_label, btn, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_label_set_recolor(wifi_label, true);
    lv_obj_add_event_cb( wifi_label,  [](lv_event_t *e) {
        lv_obj_t *label = (lv_obj_t *)lv_event_get_target(e);
        lv_msg_t *m = lv_event_get_msg(e);
        const char *v = (const char *)lv_msg_get_payload(m);
        lv_label_set_text(label, v);
    },
    LV_EVENT_MSG_RECEIVED, NULL);
    lv_msg_subsribe_obj(MSG_WIFI_UPDATE, wifi_label, NULL);
}
