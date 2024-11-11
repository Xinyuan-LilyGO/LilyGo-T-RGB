
## News

* The firmware supports any one in the list below. The firmware used in the list below is the same and there is no need to distinguish it.


## 1️⃣Support Product

| Product(PinMap)                 | SOC        | Flash | PSRAM    | Resolution | Size     | firmware                                              |
| ------------------------------- | ---------- | ----- | -------- | ---------- | -------- | ----------------------------------------------------- |
| [T-RGB 2.1 inch Half Circle][1] | ESP32-S3R8 | 16MB  | 8MB(OPI) | 480x480    | 2.1 Inch | [firmware](./LILYGO_T_RGB_2_1_INCHES_HALF_CIRCLE.bin) |
| [T-RGB 2.8 inch Full Circle][2] | ESP32-S3R8 | 16MB  | 8MB(OPI) | 480x480    | 2.1 Inch | [firmware](./LILYGO_T_RGB_2_1_INCHES_FULL_CIRCLE.bin) |
| [T-RGB 2.8 inch Full Circle][3] | ESP32-S3R8 | 16MB  | 8MB(OPI) | 480x480    | 2.8 Inch | [firmware](./LILYGO_T_RGB_2_8_INCHES.bin)             |


[1]: https://www.lilygo.cc/products/t-rgb?variant=42407295877301
[2]: https://www.lilygo.cc/products/t-rgb
[3]: https://www.lilygo.cc/products/t-rgb?variant=42880799441077

## 2️⃣How to Flash ?


### Use ESP Download Tool

- Download [Flash_download_tool](https://www.espressif.com.cn/sites/default/files/tools/flash_download_tool_3.9.7_0.zip)

![web_flasher](./images/esp_downloader.gif)

* Note that after writing is completed, you need to press RST to reset.
* When writing the keyboard firmware, you need to select ESP32-C3 in the chip type

### Use Web Flasher

- [ESP Web Flasher Online](https://espressif.github.io/esptool-js/)

![web_flasher](./images/web_flasher.gif)

* Note that after writing is completed, you need to press RST to reset.

### Use command line


If system asks about install Developer Tools, do it.

```
python3 -m pip install --upgrade pip
python3 -m pip install esptool
```

In order to launch esptool.py, exec directly with this:

```
python3 -m esptool
```

```
esptool --chip esp32s3  --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x0 firmware_file.bin

```

# 3.FAQ

- Can't upload any sketch，Please enter the upload mode manually.
   1. Connect the board via the USB cable
   2. Press and hold the **BOOT** button , While still pressing the **BOOT** button 
   3. Press **RST** button
   4. Release the **RST** button
   5. Release the **BOOT** button 
   6. Upload sketch
   7. Press the **RST** button to exit download mode
