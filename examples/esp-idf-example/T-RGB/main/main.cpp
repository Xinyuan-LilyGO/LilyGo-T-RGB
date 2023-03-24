#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "pins_config.h"
#include "sdkconfig.h"
#include "xl9535.h"
#include <stdio.h>

#include "demos/lv_demos.h"



static const char *TAG = "T-RGB example";

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define I2C_MASTER_PORT             I2C_NUM_0


// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

// #define USING_2_1_INC_FT3267 1
#define USING_2_1_INC_CST820 1
// #define USING_2_8_INC_GT911 1


#if !defined(USING_2_1_INC_CST820) && !defined(USING_2_8_INC_GT911) && !defined(USING_2_1_INC_FT3267)
#error "Please define the size of the screen and open the macro definition at the top of the sketch"
#endif

#if defined(USING_2_1_INC_FT3267)
#define TOUCH_MODULES_FT3267
#elif defined(USING_2_1_INC_CST820)
#define TOUCH_MODULES_CST_SELF
#elif defined(USING_2_8_INC_GT911)
#define TOUCH_MODULES_GT911
#endif

#include "TouchLib.h"

#if defined(USING_2_1_INC_FT3267)
#define TOUCH_SLAVE_ADDRESS FT3267_SLAVE_ADDRESS
#elif defined(USING_2_1_INC_CST820)
#define TOUCH_SLAVE_ADDRESS CST820_SLAVE_ADDRESS
#elif defined(USING_2_8_INC_GT911)
#define TOUCH_SLAVE_ADDRESS GT911_SLAVE_ADDRESS2
//!  GT911 has prober need fix !
//!  GT911 has prober need fix !
//!  GT911 has prober need fix !
#endif
TouchLib touch;


typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

#if defined(USING_2_1_INC_CST820) || defined(USING_2_1_INC_FT3267)
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 0x10},
    {0xb1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 0x0b},
    {0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    // {0xFF, {0x77, 0x01, 0x00, 0x00, 0x12}, 0x05},
    // {0xd1, {0x81}, 0x01},
    // {0xd2, {0x06}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}
};
#elif defined(USING_2_8_INC_GT911)
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xEF, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3B, 0X00}, 0x02},
    {0xC1, {0x10, 0x0C}, 0x02},
    {0xC2, {0x07, 0x0A}, 0x02},
    {0xC7, {0x00}, 0x01},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0, {0x05, 0x12, 0x98, 0x0e, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11}, 0x10},
    {0xb1, {0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x5d}, 0x01},
    {0xb1, {0x2D}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x4E}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    // {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33}, 0x0b},
    {0xe2, {0x11, 0x11, 0x33, 0x33, 0xf4, 0x00, 0x00, 0x00, 0xf4, 0x00, 0x00, 0x00}, 0x0c},
    {0xe3, {0x00, 0x00, 0x11, 0x11}, 0x04},
    {0xe4, {0x44, 0x44}, 0x02},
    {0xe5, {0x0d, 0xf5, 0x30, 0xf0, 0x0f, 0xf7, 0x30, 0xf0, 0x09, 0xf1, 0x30, 0xf0, 0x0b, 0xf3, 0x30, 0xf0}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x11}, 0x04},
    {0xe7, {0x44, 0x44}, 0x02},
    {0xe8, {0x0c, 0xf4, 0x30, 0xf0, 0x0e, 0xf6, 0x30, 0xf0, 0x08, 0xf0, 0x30, 0xf0, 0x0a, 0xf2, 0x30, 0xf0}, 0x10},
    {0xe9, {0x36}, 0x01},
    {0xeb, {0x00, 0x01, 0xe4, 0xe4, 0x44, 0x88, 0x40}, 0x07},
    {0xed, {0xff, 0x10, 0xaf, 0x76, 0x54, 0x2b, 0xcf, 0xff, 0xff, 0xfc, 0xb2, 0x45, 0x67, 0xfa, 0x01, 0xff}, 0x10},
    {0xef, {0x08, 0x08, 0x08, 0x45, 0x3f, 0x54}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},

    {0x11, {0x00}, 0x80},
    {0x3a, {0x66}, 0x01},
    {0x36, {0x08}, 0x01},
    {0x35, {0x00}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}
};
#endif

static void tft_init(void);

static int readRegister(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint8_t len)
{
    uint8_t read_data = 0;
    i2c_master_write_read_device(I2C_MASTER_PORT, devAddr, (uint8_t *)&regAddr, 1, data, len, 1000 / portTICK_PERIOD_MS);
    return 0;
}

static int writeRegister(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint8_t len)
{
    uint8_t write_buf[100] = {(uint8_t)regAddr};
    memcpy(write_buf + 1, data, len);
    i2c_master_write_to_device(I2C_MASTER_PORT, devAddr, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);
    return 0;
}

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{

    if (touch.read()) {
        TP_Point t = touch.getPoint(0);
        data->point.x = t.x;
        data->point.y = t.y;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data,
                                   void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

extern "C" void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    static lv_indev_drv_t indev_drv;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IIC_SDA_PIN,
        .scl_io_num = IIC_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master =
        {
            .clk_speed = 400 * 1000,
        },
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);

    if (err != ESP_OK) {
        ESP_LOGI(TAG, "i2c bus creation failed");
    }
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);

    ESP_LOGI(TAG, "Initialize xl9535");
    xl9535_begin(0, 0, 0, I2C_MASTER_PORT);
    xl9535_pinMode(PWR_EN_PIN, OUTPUT);
    xl9535_digitalWrite(PWR_EN_PIN, 1);

    ESP_LOGI(TAG, "Initialize touch");


    xl9535_pinMode(TP_RES_PIN, OUTPUT);
    xl9535_digitalWrite(TP_RES_PIN, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xl9535_digitalWrite(TP_RES_PIN, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xl9535_digitalWrite(TP_RES_PIN, 1);

    touch.begin(TOUCH_SLAVE_ADDRESS, -1, readRegister, writeRegister);

    ESP_LOGI(TAG, "Initialize st7701s");
    tft_init();

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT));
#endif

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings =
        {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_pulse_width = 1,
            .hsync_back_porch = 30,
            .hsync_front_porch = 50,
            .vsync_pulse_width = 1,
            .vsync_back_porch = 30,
            .vsync_front_porch = 20,
            .flags =
            {
                .pclk_active_neg = true,
            },
        },

        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width

#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .psram_trans_align = 64,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .data_gpio_nums =
        {
            // EXAMPLE_PIN_NUM_DATA12, // 2.1
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
            EXAMPLE_PIN_NUM_DATA16,
            EXAMPLE_PIN_NUM_DATA17,

            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,

            // EXAMPLE_PIN_NUM_DATA0, // 2.1
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
        },
        .flags =
        {
            .fb_in_psram = true, // allocate frame buffer in PSRAM
#if CONFIG_EXAMPLE_DOUBLE_FB
            .double_fb = true, // allocate double frame buffer
#endif                         // CONFIG_EXAMPLE_DOUBLE_FB
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Register touch driver to LVGL");
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lv_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {.callback = &example_increase_lvgl_tick, .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if LV_USE_DEMO_WIDGETS
    ESP_LOGI(TAG, "Display LVGL Demos");
    lv_demo_widgets();
#elif LV_USE_DEMO_BENCHMARK
    ESP_LOGI(TAG, "Display LVGL Demos");
    lv_demo_benchmark();
#elif LV_USE_DEMO_STRESS
    ESP_LOGI(TAG, "Display LVGL Demos");
    lv_demo_stress();
#elif LV_USE_DEMO_KEYPAD_AND_ENCODER
    ESP_LOGI(TAG, "Display LVGL Demos");
    lv_demo_keypad_encoder();
#elif LV_USE_DEMO_MUSIC
    ESP_LOGI(TAG, "Display LVGL Demos");
    lv_demo_music();
#elif LV_USE_GIF
    LV_IMG_DECLARE(rock_gif);
    ESP_LOGI(TAG, "Test decoding gif pictures. Pay attention to turn on LVGL GIF support.");
    lv_obj_t *img = lv_gif_create(lv_scr_act());
    lv_gif_set_src(img, &rock_gif);
    lv_obj_center(img);
#else
    LV_IMG_DECLARE(logo);
    ESP_LOGI(TAG, "Test decoding pictures.");
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &logo);
    lv_obj_center(img);

#endif

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(2));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();

        if (touch.read()) {
            uint8_t n = touch.getPointNum();
            printf("getPointNum: %d  \r\n", n);
            for (uint8_t i = 0; i < n; i++) {
                TP_Point t = touch.getPoint(i);
                printf("[%d] point x: %d  point y: %d \r\n", i, t.x, t.y);
            }
        }
    }
}

static void lcd_send_data(uint8_t data)
{
    uint8_t n;
    for (n = 0; n < 8; n++) {
        if (data & 0x80)
            xl9535_digitalWrite(LCD_SDA_PIN, 1);
        else
            xl9535_digitalWrite(LCD_SDA_PIN, 0);

        data <<= 1;
        xl9535_digitalWrite(LCD_CLK_PIN, 0);
        xl9535_digitalWrite(LCD_CLK_PIN, 1);
    }
}

static void lcd_cmd(const uint8_t cmd)
{
    xl9535_digitalWrite(LCD_CS_PIN, 0);
    xl9535_digitalWrite(LCD_SDA_PIN, 0);
    xl9535_digitalWrite(LCD_CLK_PIN, 0);
    xl9535_digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(cmd);
    xl9535_digitalWrite(LCD_CS_PIN, 1);
}

static void lcd_data(const uint8_t *data, int len)
{
    uint32_t i = 0;
    if (len == 0)
        return; // no need to send anything
    do {
        xl9535_digitalWrite(LCD_CS_PIN, 0);
        xl9535_digitalWrite(LCD_SDA_PIN, 1);
        xl9535_digitalWrite(LCD_CLK_PIN, 0);
        xl9535_digitalWrite(LCD_CLK_PIN, 1);
        lcd_send_data(*(data + i));
        xl9535_digitalWrite(LCD_CS_PIN, 1);
        i++;
    } while (len--);
}

static void tft_init(void)
{
    int cmd = 0;
    xl9535_pinMode(LCD_SDA_PIN, OUTPUT);
    xl9535_pinMode(LCD_CLK_PIN, OUTPUT);
    xl9535_pinMode(LCD_CS_PIN, OUTPUT);
    xl9535_pinMode(LCD_RST_PIN, OUTPUT);
    // xl9535_read_all_reg();

    xl9535_digitalWrite(LCD_SDA_PIN, 1);
    xl9535_digitalWrite(LCD_CLK_PIN, 1);
    xl9535_digitalWrite(LCD_CS_PIN, 1);

    // Reset the display
    xl9535_digitalWrite(LCD_RST_PIN, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xl9535_digitalWrite(LCD_RST_PIN, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    xl9535_digitalWrite(LCD_RST_PIN, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    while (st_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(st_init_cmds[cmd].cmd);
        lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
        if (st_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
}
