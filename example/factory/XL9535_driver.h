#pragma once

#include "Arduino.h"
#include "Wire.h"

#define XL9535_IIC_ADDRESS          0X20

#define XL9535_INPUT_PORT_0_REG     0X00
#define XL9535_INPUT_PORT_1_REG     0X01
#define XL9535_OUTPUT_PORT_0_REG    0X02
#define XL9535_OUTPUT_PORT_1_REG    0X03
#define XL9535_INVERSION_PORT_0_REG 0X04
#define XL9535_INVERSION_PORT_1_REG 0X05
#define XL9535_CONFIG_PORT_0_REG    0X06
#define XL9535_CONFIG_PORT_1_REG    0X07

class XL9535 {
public:
  XL9535(){};
  ~XL9535(){};

  void begin(bool A0 = 0, bool A1 = 0, bool A2 = 0, TwoWire *wire = &Wire);
  void pinMode(uint8_t pin, uint8_t mode);
  void pinMode8(uint8_t port, uint8_t pin, uint8_t mode);

  void digitalWrite(uint8_t pin, uint8_t val);
  int digitalRead(uint8_t pin);
  void read_all_reg();

protected:
  void writeRegister(uint8_t reg, uint8_t *data, uint8_t len);
  uint8_t readRegister(uint8_t reg, uint8_t *data, uint8_t len);

  uint8_t _address;
  TwoWire *_wire;
  bool is_found;
};