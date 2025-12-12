/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Battery
#include "gpio_bsp.h"
#include "adc_bsp.h"

//#define LV_CONF_INCLUDE_SIMPLE 1
#include "lvgl.h"
#include "demos/lv_demos.h"

#include "esp_lcd_sh8601.h"
#include "esp_lcd_touch_ft5x06.h"

#include "Shutdown/ui.h"

extern void gauge_scale();

#define TAG "LVGL-TEST"

#define Rotate_90 0
#define Rotate_None 1
#define AMOLED_Rotate Rotate_None

#if AMOLED_Rotate==Rotate_90
    #define WSBOARD_LCD_H_RES 600
    #define WSBOARD_LCD_V_RES 450
#else
    #define WSBOARD_LCD_H_RES 450
    #define WSBOARD_LCD_V_RES 600
#endif

#define EXAMPLE_LVGL_BUF_HEIGHT        (WSBOARD_LCD_H_RES/10)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

#define LCD_BIT_PER_PIXEL       (16)

static esp_lcd_touch_handle_t tp;

void init_lcd();
void init_touch();
void init_lvgl();

lv_draw_buf_t disp_buf;
lv_display_t *disp_drv;
esp_lcd_panel_handle_t panel_handle;

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        task_delay_ms = lv_timer_handler();
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static LV_ATTRIBUTE_FAST_MEM uint32_t my_tick_get_cb(void)
{
    return esp_timer_get_time() / 1000;
}

void app_main(void)
{
    init_lcd();
    init_touch();

    // Battery
    esp32_gpio_init();

    init_lvgl();

    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    printf("CPU frequency: %d MHz\n", esp_clk_cpu_freq() / 1000000);

    //lv_demo_widgets();
    //lv_demo_benchmark();
    //lv_demo_music();

    gauge_scale();

    //ui_init();

#if 0
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#endif

#if 0
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
#endif

    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    // Lock the mutex due to the LVGL APIs are not thread-safe
 //   if (example_lvgl_lock(-1))
 //   {
        //lv_demo_widgets();          /* A widgets example */
        //lv_demo_music();           /* A modern, smartphone-like music player demo. */
        //lv_demo_stress();          /* A stress test for LVGL. */
        //lv_demo_benchmark();      /* A demo to measure the performance of LVGL or to compare different settings. */
        // Release the mutex
       // example_lvgl_unlock();
 //   }
}


static LV_ATTRIBUTE_FAST_MEM void example_lvgl_flush_cb(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(drv);
#if (AMOLED_Rotate == Rotate_90)
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1 + 16;
    const int offsety2 = area->y2 + 16;
#else
    const int offsetx1 = area->x1 + 16;
    const int offsetx2 = area->x2 + 16;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;
#endif
#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    // Special dealing for first pixel
    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;
    // Normal dealing for other pixels
    for (int i = 1; i < pixel_num; i++) {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    // lv_draw_sw_rgb565_swap(color_p, w * h);
    lv_draw_sw_rgb565_swap((lv_color_t *)color_map, (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1));

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    lv_display_flush_ready(drv);
}

#if 0
static void disp_flush(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *color_p)
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    auto *plane = (LilyGo_Display *)lv_display_get_user_data(disp_drv);
    lv_draw_sw_rgb565_swap(color_p, w * h);
    plane->pushColors(area->x1, area->y1, w, h, (uint16_t *)color_p);
    lv_display_flush_ready( disp_drv );
}
#endif

static LV_ATTRIBUTE_FAST_MEM void example_lvgl_rounder_cb(lv_event_t *e) {
    lv_area_t *area = (lv_area_t *)lv_event_get_param(e);

    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

static LV_ATTRIBUTE_FAST_MEM void touchpad_read( lv_indev_t *indev, lv_indev_data_t *data )
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    uint16_t tp_x;
    uint16_t tp_y;
    uint8_t tp_cnt = 0;
    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);
    /* Read data from touch controller */
    bool tp_pressed = esp_lcd_touch_get_coordinates(tp, &tp_x, &tp_y, NULL, &tp_cnt, 1);
    if (tp_pressed && tp_cnt > 0) {
        /* Set the coordinates and state */
        data->point.x = tp_x;
        data->point.y = tp_y;
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

void init_lvgl() {
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    disp_drv = lv_display_create(WSBOARD_LCD_H_RES, WSBOARD_LCD_V_RES);


    ESP_LOGI(TAG, "Initialize display buffer");
    const uint32_t buf_size_in_bytes = WSBOARD_LCD_H_RES * WSBOARD_LCD_V_RES * sizeof(lv_color_t) / 10;
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(buf_size_in_bytes, MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(buf_size_in_bytes, MALLOC_CAP_DMA);
    assert(buf2);
    lv_display_set_buffers(disp_drv, buf1, buf2, buf_size_in_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp_drv, example_lvgl_flush_cb);
    lv_display_add_event_cb(disp_drv, example_lvgl_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    lv_display_set_color_format(disp_drv, LV_COLOR_FORMAT_RGB565);
    lv_display_set_user_data(disp_drv, panel_handle);

    lv_indev_t *indev_drv = lv_indev_create();
    lv_indev_set_type(indev_drv, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_drv, touchpad_read);
    lv_indev_set_user_data(indev_drv, tp);
    lv_indev_enable(indev_drv, true);
    lv_indev_set_display(indev_drv, disp_drv);

    lv_tick_set_cb(my_tick_get_cb);

    lv_group_set_default(lv_group_create());

#if AMOLED_Rotate==Rotate_90
    // Rotate screen 180 degrees
    // lv_display_set_rotation(disp_drv, LV_DISPLAY_ROTATION_180);
#endif
    // lv_tick_set_cb();
}

static LV_ATTRIBUTE_FAST_MEM bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_display_flush_ready(disp_drv);
    return false;
}

#define LCD_HOST    SPI2_HOST

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t []){0x20}, 1, 0},	
    {0x26, (uint8_t []){0x0A}, 1, 0}, 
    {0x24, (uint8_t []){0x80}, 1, 0}, 

    {0xFE, (uint8_t []){0x00}, 1, 0},    
    {0x3A, (uint8_t []){0x55}, 1, 0}, //Interface Pixel Format	16bit/pixel     
    {0xC2, (uint8_t []){0x00}, 1, 10},  
    {0x35, (uint8_t []){0x00}, 0, 0}, //TE ON
    {0x51, (uint8_t []){0x00}, 1, 10}, //Write Display Brightness MAX_VAL=0XFF
    {0x11, (uint8_t []){0x00}, 0, 80},  
    {0x2A, (uint8_t []){0x00,0x10,0x01,0xD1}, 4, 0},
    {0x2B, (uint8_t []){0x00,0x00,0x02,0x57}, 4, 0},
    // {0x30, (uint8_t []){0x00, 0x01,0x02, 0x56}, 4, 0},   
    {0x29, (uint8_t []){0x00}, 0, 10},
#if (AMOLED_Rotate == Rotate_90)
    {0x36, (uint8_t []){0x30}, 1, 10},
#endif
    {0x51, (uint8_t []){0xFF}, 1, 0},//Write Display Brightness MAX_VAL=0XFF
};

#define EXAMPLE_PIN_NUM_LCD_CS            (GPIO_NUM_9)
#define EXAMPLE_PIN_NUM_LCD_PCLK          (GPIO_NUM_10) 
#define EXAMPLE_PIN_NUM_LCD_DATA0         (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA1         (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA2         (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_DATA3         (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_RST           (GPIO_NUM_21)

void init_lcd() {
    ESP_LOGI(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA0,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA1,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA2,
                                                                 EXAMPLE_PIN_NUM_LCD_DATA3,
                                                                 WSBOARD_LCD_H_RES * WSBOARD_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                                notify_lvgl_flush_ready,
                                                                                NULL);
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "LCD initialized");
}

#define TOUCH_HOST I2C_NUM_0

#define WSBOARD_PIN_NUM_TOUCH_SDA GPIO_NUM_47
#define WSBOARD_PIN_NUM_TOUCH_SCL GPIO_NUM_48
#define WSBOARD_PIN_NUM_TOUCH_RST GPIO_NUM_3
#define WSBOARD_PIN_NUM_TOUCH_INT (-1)

void init_touch() {
    ESP_LOGI(TAG, "Initialize I2C bus");
    const i2c_config_t i2c_conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = WSBOARD_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = WSBOARD_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 300 * 1000,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0));
    
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();

    // Attach the TOUCH to the I2C bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle));

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = WSBOARD_LCD_V_RES-1,
        .y_max = WSBOARD_LCD_H_RES-1,
        .rst_gpio_num = WSBOARD_PIN_NUM_TOUCH_RST,
        .int_gpio_num = WSBOARD_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
#if (AMOLED_Rotate == Rotate_90)
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 1,
#else
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
#endif
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));
}

