/*
 * This program reads temperature from BMP280 via Grove connector (i2c mode) and prints it on LilyGo T-RGB screen
 * Эта программа читает температуру с BMP280 через Grove-разъём (i2c) и выводит её на экран LilyGo T-RGB
 * 
 * It requires Adafruit_BMP280 library and all the libraries that are required by LilyGo T-RGB library.
 * Она требует наличия библиотеки Adafruit_BMP280 и всех библиотек необходимых для LilyGo T-RGB
 * LilyGo T-RGB repository (setup instructions are there): https://github.com/Xinyuan-LilyGO/LilyGo-T-RGB
 * 
 * Mautoz Tech - https://www.youtube.com/c/MautozTech
 * Заметки ESPшника - https://www.youtube.com/@espdev
 */
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

#include <LilyGo_RGBPanel.h>
#include <LV_Helper.h>
LilyGo_RGBPanel panel;
lv_obj_t *label;

void setup() {
  Serial.begin(9600);
  Wire.begin(8, 48);
  unsigned status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    while (1) delay(10);
  }

  if (!panel.begin()) {
    while (1) { Serial.println("Error, failed to initialize T-RGB"); delay(1000);}
  }
  beginLvglHelper(panel);
  label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Starting...");
  lv_obj_center(label);   
  lv_obj_set_style_text_font(label, &lv_font_montserrat_36, 0);
  panel.setBrightness(16);
}

void loop() {
    Serial.println("Temperature: "+String(bmp.readTemperature()) + " *C");
    Serial.println("Pressure: "+String(bmp.readPressure()) + " *Pa");
    Serial.println("Approx altitude: "+String(bmp.readAltitude(1013.25)) + " m");
    Serial.println();

    lv_label_set_text(label, ("Temperature: " + String(bmp.readTemperature()) + " *C").c_str());  
    lv_timer_handler();
    delay(2000);
}
