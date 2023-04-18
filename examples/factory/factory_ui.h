#pragma once

#define MSG_BAT_VOLT_UPDATE 1
#define MSG_TOUCH_UPDATE    2
#define MSG_WIFI_UPDATE     3
#define MSG_TOUCH_INT_UPDATE     4

#include "stdint.h"

typedef struct {
    uint16_t x;
    uint16_t y;
} touch_point_t;

void ui_begin();

#define FT5x06_ADDR 0x38
#define CST820_ADDR 0x15
#define GT911_ADDR  0x5A

const char *getTouchAddr();