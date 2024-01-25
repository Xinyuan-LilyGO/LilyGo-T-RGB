/**
 * @file      LilyGo_Display.h
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-01-24
 *
 */
#pragma once

#include <stdint.h>

class LilyGo_Display
{
public:
    LilyGo_Display(): _rotation(0) {};
    virtual void pushColors(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *data) = 0;
    virtual uint16_t  width() = 0;
    virtual uint16_t  height() = 0;
    virtual uint8_t getPoint(int16_t *x, int16_t *y, uint8_t get_point ) = 0;
protected:
    uint8_t _rotation;
};
