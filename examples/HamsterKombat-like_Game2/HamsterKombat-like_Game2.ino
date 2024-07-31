/*
 * Эта игра типа Hamster Combat показывает как программировать ESP c экраном LilyGo T-RGB: как управлять программой с помощью сенсорного экрана, как позиционировать объекты и т.д.
 * HamsterCombat-like game for LilyGo T-RGB. It shows you how to program the screen using LVGL library: how to control a program with touchscreen, how to position objects, etc
 * 
 * LilyGo T-RGB repository (Setup manual is there): https://github.com/Xinyuan-LilyGO/LilyGo-T-RGB
 * 
 * Mautoz Tech - https://www.youtube.com/c/MautozTech
 * Заметки ESPшника - https://www.youtube.com/@espdev
 */
#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
LilyGo_RGBPanel panel;

const String FILENAME = "/hamster.jpg";
lv_obj_t *score_label;
int16_t x, y;
uint8_t cnt = 0;
bool previousState = false;

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
    lv_obj_t *img = lv_img_create(lv_scr_act()); // Создание обьекта картинки
    lv_obj_center(img); // Установка картинки в центр экрана
    String path = lvgl_helper_get_fs_filename(FILENAME); // Изменение формата пути картинки
    lv_img_set_src((lv_obj_t *)img, path.c_str()); // Чтение картинки с карты памяти и установка на экран
    //lv_obj_set_size(img, 240, 240);

    lv_obj_t *label = lv_label_create(lv_scr_act()); // Содание текстового обьекта
    lv_label_set_text(label, "Hamster Combat"); // Установка текста "Hamster Combat"      
    lv_obj_align_to(label, img, LV_ALIGN_OUT_TOP_MID, 0, -30); // Установка текста (label) сверху картинки (img) со смещением на -30 пикселей (30 пикс. вверх)
    //lv_obj_add_event_cb(img, btn_event_cb, LV_EVENT_ALL, NULL);  
    
    lv_obj_t *btn = lv_btn_create(lv_scr_act()); // Создание обьекта кнопки, которая по сути будет являться рамкой для текста с количеством очков   
    lv_obj_set_size(btn, 120, 50); // Установка размера кнопки       
    lv_obj_align_to(btn, img, LV_ALIGN_OUT_BOTTOM_MID, 0, -40); // Установка кнопки (btn) под картинкой (img) со смещением на -40 пикселей (40 пикс. вверх)

    score_label = lv_label_create(btn); // Установка текста на кнопку     
    lv_label_set_text(score_label, "Score: 0"); // Установка текста "Score: 0"        
    lv_obj_center(score_label); // Установка текста по центру кнопки

    const char *touchModel =  panel.getTouchModelName(); // Получение обьекта сенсора экрана
    lv_task_handler();
    
    panel.setBrightness(16); // Установка яркости дисплея 16 (0 - выкл, 1 - мин. яркость, 16 - макс. яркость)
}

void loop(){
    bool touched = panel.getPoint(&x, &y); // Touched = "true" если нажатие на экран было в этом цикле, "false" если нажатия не было. Координаты нажатия сохраняются в X и Y   
    if (touched and !previousState) { // Если на экран нажали, а до этого он не был нажат
        if (x > 120 and x < 360 and y > 120 and y < 360){ // Если кликнули именно на координату картинки
          cnt++; // Прибавление 1 к счётчику кликов 
          lv_label_set_text_fmt(score_label, "Score: %d", cnt); // Замена количества кликов на экране
          previousState = true; // Смена переменной предыдущего состояния, т.к. в этом цикле кнопка была нажата
        }
    }
    else if (!touched and previousState){ // Если нажатия не было в этом цикле, но было в предыдущем, то возвращаем значение предыдущего состояния "false"
      previousState = false;  
    }
    lv_task_handler();
    delay(9); // 9 мс - идеальная задержка. Информация с сенсора успевает обновляться, и нет лагов
}
