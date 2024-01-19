<h1 align = "center"> 🌟T-RGB🌟</h1>

## News

1. Easy to use RGB panel library, organized by [fablabnbg/TRGBArduinoSupport](https://github.com/fablabnbg/TRGBArduinoSupport),This is more recommended for novices (**Only Support 2.1 inch FT3267 touch chip version**)
2. Currently, there are three types of RGB panels
    * 2.1 inch FT3267 touch chip version (Half circle 2.1 inches use FT3267 touch screen)
    * 2.1 inch CST820 touch chip version (Full circle 2.1 inches using CST820 touch screen)
    * 2.8 inch GT911 touch chip version (Full circle 2.8 inches using GT911 touch screen)

## Description

`T-RGB` is a development board whose main control chip is ESP32-S3. It is equipped with a `2.1-inch LCD` color screen (3SPI&RGB interface) and  touch chip. It has TF card reading and writing function and is equipped with a programmable button and a set of IIC pins. It can be powered with a lithium battery and charged with a USB. You can use the ESP32S3 directly for USB communication or programming.

![specifications_en](assets/image/specifications_en.jpg)

## Pinmap

![pinmap_en](assets/image/pinmap_en.jpg)

## Product 📷

| Product |                            Product Link                            |
| :-----: | :----------------------------------------------------------------: |
|  T-RGB  | [AliExpress](https://www.aliexpress.us/item/1005004778542414.html) |

## Quick Start

### PlatformIO Quick Start (Recommended)

1. Install [Visual Studio Code](https://code.visualstudio.com/) and [Python](https://www.python.org/)
2. Search for the `PlatformIO` plugin in the `VisualStudioCode` extension and install it.
3. After the installation is complete, you need to restart `VisualStudioCode`
4. After restarting `VisualStudioCode`, select `File` in the upper left corner of `VisualStudioCode` -> `Open Folder` -> select the `T-RGB` directory
5. Wait for the installation of third-party dependent libraries to complete
6. Click on the `platformio.ini` file, and in the `platformio` column
7. Uncomment one of the lines `default_envs = xxxx` to make sure only one line works
8. Click the (✔) symbol in the lower left corner to compile
9. Connect the board to the computer USB
10. Click (→) to upload firmware
11. Click (plug symbol) to monitor serial output
12. If it cannot be written, or the USB device keeps flashing, please check the **FAQ** below

### Arduino IDE Quick Start

* It is recommended to use platformio without cumbersome steps

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install [Arduino ESP32 V 2.0.5 or above and below V3.0](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
3. Download `T-RGB` to Decktop
4. Copy all folders in [lib folder](./lib/)  to Arduino library folder (e.g. C:\Users\YourName\Documents\Arduino\libraries)
5. Open ArduinoIDE  ,`Tools` , Look at the picture to choose
  ![setting](images/ArduinoIDE.jpg)
1. Open `T-RGB` -> `examples` -> `any examples` -> `any eaxmples.ino`
2. Select `Port`
3. Click `upload` , Wait for compilation and writing to complete
4. If it cannot be written, or the USB device keeps flashing, please check the **FAQ** below


### ESP-IDF:
- The installation method is also inconsistent depending on the system, it is recommended to refer to the [official manual](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for installation

## FAQ

1. The board uses USB as the JTAG upload port. When printing serial port information on USB_CDC_ON_BOOT configuration needs to be turned on.
If the port cannot be found when uploading the program or the USB has been used for other functions, the port does not appear.
Please enter the upload mode manually.
   1. Connect the board via the USB cable
   2. Press and hold the BOOT button , While still pressing the BOOT button, press RST
   3. Release the RST
   4. Release the BOOT button
   5. Upload sketch
