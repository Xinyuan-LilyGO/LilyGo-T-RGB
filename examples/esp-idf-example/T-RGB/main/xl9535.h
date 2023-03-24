#ifndef __XL9535_H__
#define __XL9535_H__

#include "hal/i2c_types.h"
#include <stdio.h>

#define XL9535_IIC_ADDRESS          0X20

#define XL9535_INPUT_PORT_0_REG     0X00
#define XL9535_INPUT_PORT_1_REG     0X01
#define XL9535_OUTPUT_PORT_0_REG    0X02
#define XL9535_OUTPUT_PORT_1_REG    0X03
#define XL9535_INVERSION_PORT_0_REG 0X04
#define XL9535_INVERSION_PORT_1_REG 0X05
#define XL9535_CONFIG_PORT_0_REG    0X06
#define XL9535_CONFIG_PORT_1_REG    0X07

#define INPUT                       0x01
#define OUTPUT                      0x03

#ifdef __cplusplus
extern "C" {
#endif

void xl9535_begin(uint8_t A0, uint8_t A1, uint8_t A2, i2c_port_t i2c_port);
void xl9535_pinMode(uint8_t pin, uint8_t mode);
void xl9535_pinMode8(uint8_t port, uint8_t pin, uint8_t mode);

void xl9535_digitalWrite(uint8_t pin, uint8_t val);
int xl9535_digitalRead(uint8_t pin);
void xl9535_read_all_reg();

#ifdef __cplusplus
}
#endif

#endif