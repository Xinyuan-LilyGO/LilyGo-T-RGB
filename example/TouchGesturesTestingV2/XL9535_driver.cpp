#include "XL9535_driver.h"
#include "Wire.h"

void XL9535::writeRegister(uint8_t reg, uint8_t *data, uint8_t len) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  for (uint8_t i = 0; i < len; i++) {
    _wire->write(data[i]);
  }
  _wire->endTransmission();
}
uint8_t XL9535::readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_address, len);
  uint8_t index = 0;
  while (index < len)
    data[index++] = _wire->read();
  return 0;
}

void XL9535::begin(bool A0, bool A1, bool A2, TwoWire *wire) {
  _address = XL9535_IIC_ADDRESS | (A2 << 3) | (A1 << 2) | (A0 << 1);
  _wire = wire;
  is_found = true;
  _wire->beginTransmission(_address);
  if (!_wire->endTransmission()) {
    Serial.println("Found xl9535");
  } else {
    Serial.println("xl9535 not found");
    is_found = false;
  }
}
void XL9535::pinMode(uint8_t pin, uint8_t mode) {
  if (is_found) {
    uint8_t port = 0;
    if (pin > 7) {
      readRegister(XL9535_CONFIG_PORT_1_REG, &port, 1);
      if (mode == OUTPUT) {
        port = port & (~(1 << (pin - 10)));
      } else {
        port = port | (1 << (pin - 10));
      }
      writeRegister(XL9535_CONFIG_PORT_1_REG, &port, 1);

    } else {
      readRegister(XL9535_CONFIG_PORT_0_REG, &port, 1);
      if (mode == OUTPUT) {
        port = port & (~(1 << pin));
      } else {
        port = port | (1 << pin);
      }
      writeRegister(XL9535_CONFIG_PORT_0_REG, &port, 1);
    }
  } else {
    Serial.println("xl9535 not found");
  }
}
void XL9535::pinMode8(uint8_t port, uint8_t pin, uint8_t mode) {
  if (is_found) {
    uint8_t _pin = (mode != OUTPUT) ? pin : ~pin;
    if (port) {
      writeRegister(XL9535_CONFIG_PORT_1_REG, &_pin, 1);
    } else {
      writeRegister(XL9535_CONFIG_PORT_0_REG, &_pin, 1);
    }
  } else {
    Serial.println("xl9535 not found");
  }
}

void XL9535::digitalWrite(uint8_t pin, uint8_t val) {
  if (is_found) {
    uint8_t port = 0;
    uint8_t reg_data = 0;
    if (pin > 7) {
      readRegister(XL9535_OUTPUT_PORT_1_REG, &reg_data, 1);
      reg_data = reg_data & (~(1 << (pin - 10)));
      port = reg_data | val << (pin - 10);
      writeRegister(XL9535_OUTPUT_PORT_1_REG, &port, 1);
    } else {
      readRegister(XL9535_OUTPUT_PORT_0_REG, &reg_data, 1);
      reg_data = reg_data & (~(1 << pin));
      port = reg_data | val << pin;
      writeRegister(XL9535_OUTPUT_PORT_0_REG, &port, 1);
    }
  } else {
    Serial.println("xl9535 not found");
  }
}

int XL9535::digitalRead(uint8_t pin) {
  if (is_found) {
    int state = 0;
    uint8_t port = 0;
    if (pin > 7) {
      readRegister(XL9535_INPUT_PORT_1_REG, &port, 1);
      state = port & (pin - 10) ? 1 : 0;
    } else {
      readRegister(XL9535_INPUT_PORT_0_REG, &port, 1);
      state = port & pin ? 1 : 0;
    }
    return state;
  } else {
    Serial.println("xl9535 not found");
  }
  return 0;
}

void XL9535::read_all_reg() {
  uint8_t data;
  for (uint8_t i = 0; i < 8; i++) {
    readRegister(i, &data, 1);
    Serial.printf("0x%02x : 0x%02X \r\n", i, data);
  }
}