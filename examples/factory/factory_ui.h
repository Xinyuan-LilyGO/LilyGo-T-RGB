#pragma once

#define MSG_BAT_VOLT_UPDATE 1
#define MSG_TOUCH_UPDATE    2
#define MSG_WIFI_UPDATE     3

#include "stdint.h"

typedef struct {
  uint16_t x;
  uint16_t y;
} touch_point_t;

void ui_begin();