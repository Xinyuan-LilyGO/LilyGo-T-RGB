#include "xl9535.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"

static uint8_t _address;
static bool is_found;
static i2c_port_t _i2c_port;

static void xl9535_writeRegister(uint8_t reg, uint8_t data) {
  uint8_t write_buf[2] = {reg, data};
  i2c_master_write_to_device(_i2c_port, _address, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

static uint8_t xl9535_readRegister(uint8_t reg) {
  uint8_t read_data = 0;
  i2c_master_write_read_device(_i2c_port, _address, &reg, 1, &read_data, 1, 1000 / portTICK_PERIOD_MS);
  return read_data;
}

void xl9535_begin(uint8_t A0, uint8_t A1, uint8_t A2, i2c_port_t i2c_port) {
  esp_err_t ret;
  _address = XL9535_IIC_ADDRESS | (A2 << 2) | (A1 << 1) | (A0 << 0);
  _i2c_port = i2c_port;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK)
    is_found = 1;

  /* Initialize xl9535 default register parameters */
  xl9535_writeRegister(XL9535_OUTPUT_PORT_0_REG, 0xFF);
  xl9535_writeRegister(XL9535_OUTPUT_PORT_1_REG, 0xFF);
  xl9535_writeRegister(XL9535_INVERSION_PORT_0_REG, 0x00);
  xl9535_writeRegister(XL9535_INVERSION_PORT_1_REG, 0x00);
  xl9535_writeRegister(XL9535_CONFIG_PORT_0_REG, 0xFF);
  xl9535_writeRegister(XL9535_CONFIG_PORT_1_REG, 0xFF);
}

void xl9535_pinMode(uint8_t pin, uint8_t mode) {
  if (is_found) {
    uint8_t port = 0;
    if (pin > 7) {
      port = xl9535_readRegister(XL9535_CONFIG_PORT_1_REG);
      if (mode == OUTPUT)
        port = port & (~(1 << (pin - 10)));
      else
        port = port | (1 << (pin - 10));
      xl9535_writeRegister(XL9535_CONFIG_PORT_1_REG, port);
    } else {
      port = xl9535_readRegister(XL9535_CONFIG_PORT_0_REG);
      if (mode == OUTPUT) {
        port = port & (uint8_t)(~(1 << pin));
      } else {
        port = port | (uint8_t)(1 << pin);
      }
      xl9535_writeRegister(XL9535_CONFIG_PORT_0_REG, port);
    }
  }
}

void xl9535_pinMode8(uint8_t port, uint8_t pin, uint8_t mode) {
  uint8_t _pin = (mode != OUTPUT) ? pin : ~pin;
  if (is_found) {
    if (port) {
      xl9535_writeRegister(XL9535_CONFIG_PORT_1_REG, _pin);
    } else {
      xl9535_writeRegister(XL9535_CONFIG_PORT_0_REG, _pin);
    }
  }
}

void xl9535_digitalWrite(uint8_t pin, uint8_t val) {
  if (is_found) {
    uint8_t port = 0;
    uint8_t reg_data = 0;
    if (pin > 7) {
      reg_data = xl9535_readRegister(XL9535_OUTPUT_PORT_1_REG);
      reg_data = reg_data & (uint8_t)(~(1 << (pin - 10)));
      port = reg_data | val << (pin - 10);
      xl9535_writeRegister(XL9535_OUTPUT_PORT_1_REG, port);
    } else {
      reg_data = xl9535_readRegister(XL9535_OUTPUT_PORT_0_REG);
      // printf("1 reg_data 0x%02x   ", reg_data);
      reg_data = reg_data & (uint8_t)(~(1 << pin));
      // printf("2 reg_data 0x%02x   ", reg_data);
      port = reg_data | (val << pin);
      // printf("port 0x%02x   ", port);
      xl9535_writeRegister(XL9535_OUTPUT_PORT_0_REG, port);
    }
  }
}

int xl9535_digitalRead(uint8_t pin) {
  if (is_found) {
    int state = 0;
    uint8_t port = 0;
    if (pin > 7) {
      port = xl9535_readRegister(XL9535_INPUT_PORT_1_REG);
      state = port & (pin - 10) ? 1 : 0;
    } else {
      port = xl9535_readRegister(XL9535_INPUT_PORT_0_REG);
      state = port & pin ? 1 : 0;
    }
    return state;
  }
  return 0;
}

void xl9535_read_all_reg() {
  uint8_t data;
  for (uint8_t i = 0; i < 8; i++) {
    data = xl9535_readRegister(i);
    printf("0x%02x : 0x%02X \r\n", i, data);
  }
}