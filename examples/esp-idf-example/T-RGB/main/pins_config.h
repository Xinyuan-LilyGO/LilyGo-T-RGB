#ifndef __PINS_CONFIG_H__
#define __PINS_CONFIG_H__

#define WIFI_SSID                      "xinyuandianzi"
#define WIFI_PASSWORD                  "AA15994823428"

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (8 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       46
#define EXAMPLE_PIN_NUM_HSYNC          47
#define EXAMPLE_PIN_NUM_VSYNC          41
#define EXAMPLE_PIN_NUM_DE             45
#define EXAMPLE_PIN_NUM_PCLK           42
// #define EXAMPLE_PIN_NUM_DATA0          44
#define EXAMPLE_PIN_NUM_DATA1          21
#define EXAMPLE_PIN_NUM_DATA2          18
#define EXAMPLE_PIN_NUM_DATA3          17
#define EXAMPLE_PIN_NUM_DATA4          16
#define EXAMPLE_PIN_NUM_DATA5          15
#define EXAMPLE_PIN_NUM_DATA6          14
#define EXAMPLE_PIN_NUM_DATA7          13
#define EXAMPLE_PIN_NUM_DATA8          12
#define EXAMPLE_PIN_NUM_DATA9          11
#define EXAMPLE_PIN_NUM_DATA10         10
#define EXAMPLE_PIN_NUM_DATA11         9
// #define EXAMPLE_PIN_NUM_DATA12         43
#define EXAMPLE_PIN_NUM_DATA13         7
#define EXAMPLE_PIN_NUM_DATA14         6
#define EXAMPLE_PIN_NUM_DATA15         5
#define EXAMPLE_PIN_NUM_DATA16         3
#define EXAMPLE_PIN_NUM_DATA17         2
#define EXAMPLE_PIN_NUM_DISP_EN        -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              480

#define IIC_SCL_PIN                    48
#define IIC_SDA_PIN                    8

#define SD_CLK_PIN                     39
#define SD_CMD_PIN                     40
#define SD_D0_PIN                      38

#define BAT_VOLT_PIN                   4
#define TP_INT_PIN                     1

#define BOOT_BTN_PIN                   0

/* XL9535 --- PIN - P0*/
#define TP_RES_PIN                     1
#define PWR_EN_PIN                     2
#define LCD_CS_PIN                     3
#define LCD_SDA_PIN                    4
#define LCD_CLK_PIN                    5
#define LCD_RST_PIN                    6
#define SD_CS_PIN                      7

#endif /* __PINS_CONFIG__ */