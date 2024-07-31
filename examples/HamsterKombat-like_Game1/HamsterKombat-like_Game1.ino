/*
 * Эта игра типа Hamster Combat показывает как программировать ESP c экраном LilyGo T-RGB: как использовать кликабельные картинки, как позиционировать объекты и т.д.
 * Эта программа работает медленнее чем вторая программа, вторая программа вместо проверки нажатий на кликабельную картинку проверяет нажатие на опред. область на экране в цикле loop
 * 
 * HamsterCombat-like game for LilyGo T-RGB. It shows you how to program the screen using LVGL library: how to use clickable images, how to position objects, etc
 * This program works slower then the second program. Second program checks clicks on determined area on the screen in "loop" function and not clicks on clickable image
 * 
 * LilyGo T-RGB repository (Setup manual is there): https://github.com/Xinyuan-LilyGO/LilyGo-T-RGB
 * 
 * Mautoz Tech - https://www.youtube.com/c/MautozTech
 * Заметки ESPшника - https://www.youtube.com/@espdev
 */

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
LilyGo_RGBPanel panel;

// ИМЯ КАРТИНКИ ПО КОТОРОЙ НАДО КЛИКАТЬ (НА КАРТЕ ПАМЯТИ)
// NAME OF THE IMAGE ON WHICH YOU CLICK (PUT ON MEMORY CARD)
const String FILENAME = "/hamster.jpg";

lv_obj_t *score_label; // Обьект текстового поля, на которое выводится кол-во очков
uint8_t cnt = 0; // Счётчик кликов

void btn_event_cb(lv_event_t *e) // Эта функция выполняется при 
{
  cnt++; // Увеличение счётчика кликов на 1
  lv_label_set_text_fmt(score_label, "Score: %d", cnt); // Изменение количества очков в текстовом поле
}

void setup()
{
    Serial.begin(115200);
    if (!panel.begin()) {
        while (1) {
            Serial.println("Error, failed to initialize T-RGB"); delay(1000);
        }
    }
    if (!panel.installSD()) {
        Serial.println("Can't install SD Card!");
    }
    beginLvglHelper(panel);
    
    static lv_style_t style; // Создание обьекта стиля
    lv_style_init(&style); // Инициализация обьекта стиля
    lv_style_set_bg_color(&style, lv_color_white()); // Установка белого фона для стиля
    lv_obj_add_style(lv_scr_act(), &style, LV_PART_MAIN); // Установка стиля для экрана   

    if (!SD_MMC.exists(FILENAME)) { // Проверка существования картинки на карте памяти
       Serial.println("File not find image..."); 
       return;
    }
    lv_obj_t *img = lv_imgbtn_create(lv_scr_act()); // Создание кликабельной картинки (imgbtn - Image-button)
    lv_obj_center(img); // Установка картинки в центр экрана
    String path = lvgl_helper_get_fs_filename(FILENAME); // Изменение формата пути картинки
    lv_img_set_src((lv_obj_t *)img, path.c_str()); // Чтение картинки с карты памяти и установка на экран
    lv_obj_set_size(img, 240, 240); // Установка размера картинки: 240 на 240 пикселей

    lv_obj_t *label = lv_label_create(lv_scr_act()); // Содание текстового обьекта
    lv_label_set_text(label, "Hamster Combat"); // Установка текста "Hamster Combat"                   
    lv_obj_align_to(label, img, LV_ALIGN_OUT_TOP_MID, 0, -30); // Установка текста (label) сверху картинки (img) со смещением на -30 пикселей (30 пикс. вверх)
    lv_obj_add_event_cb(img, btn_event_cb, LV_EVENT_CLICKED, NULL); // Привязка функции btn_event_cb к картинке img
    
    lv_obj_t *btn = lv_btn_create(lv_scr_act()); // Создание кнопки, которая по сути будет являться рамкой для текста с количеством очков          
    lv_obj_set_size(btn, 120, 50); // Установка размера кнопки                          
    lv_obj_align_to(btn, img, LV_ALIGN_OUT_BOTTOM_MID, 0, -40); // Установка кнопки (btn) под картинкой (img) со смещением на -40 пикселей (40 пикс. вверх)

    score_label = lv_label_create(btn); // Установка текста на кнопку         
    lv_label_set_text(score_label, "Score: 0"); // Установка текста "Score: 0"            
    lv_obj_center(score_label); // Установка текста по центру кнопки
    
    panel.setBrightness(16); // Установка яркости дисплея 16 (0 - выкл, 1 - мин. яркость, 16 - макс. яркость)
}


void loop()
{
    lv_timer_handler(); // Обработчик LVGL
    delay(2);
}
