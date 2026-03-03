#include "display.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "display";
static const int cs_pins[LCD_COUNT] = {
    PIN_LCD1_CS, PIN_LCD2_CS, PIN_LCD3_CS,
    PIN_LCD4_CS, PIN_LCD5_CS, PIN_LCD6_CS
};
static spi_device_handle_t spi_dev;

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_LCD_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    spi_device_polling_transmit(spi_dev, &t);
}

static void lcd_data(const uint8_t *data, int len)
{
    if (len <= 0) return;
    gpio_set_level(PIN_LCD_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = data };
    spi_device_polling_transmit(spi_dev, &t);
}

static void lcd_data_byte(uint8_t val) { lcd_data(&val, 1); }

static void select_tube(int i)
{
    for (int n = 0; n < LCD_COUNT; n++)
        gpio_set_level(cs_pins[n], (n == i) ? 0 : 1);
}

static void deselect_all(void)
{
    for (int i = 0; i < LCD_COUNT; i++) gpio_set_level(cs_pins[i], 1);
}

static void st7735_init_one(int tube)
{
    select_tube(tube);
    gpio_set_level(PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x3A); lcd_data_byte(0x05);
    lcd_cmd(0x36); lcd_data_byte(0x08);
    uint8_t fr[] = {0x01, 0x2C, 0x2D};
    lcd_cmd(0xB1); lcd_data(fr, 3);
    lcd_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(50));
    deselect_all();
}

void display_init(void)
{
    ESP_LOGI(TAG, "Initialising 6x ST7735 displays");
    gpio_config_t io = { .mode = GPIO_MODE_OUTPUT };
    for (int i = 0; i < LCD_COUNT; i++) {
        io.pin_bit_mask = 1ULL << cs_pins[i]; gpio_config(&io); gpio_set_level(cs_pins[i], 1);
    }
    io.pin_bit_mask = (1ULL << PIN_LCD_DC) | (1ULL << PIN_LCD_RST); gpio_config(&io);

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_LCD_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_LCD_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = LCD_WIDTH*LCD_HEIGHT*2,
    };
    spi_bus_initialize(HSPI_HOST, &bus, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 40*1000*1000, .mode = 0, .spics_io_num = -1, .queue_size = 7,
    };
    spi_bus_add_device(HSPI_HOST, &dev, &spi_dev);

    ledc_timer_config_t tmr = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tmr);
    ledc_channel_config_t ch = {
        .gpio_num = PIN_LCD_BACKLIGHT, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 128,
    };
    ledc_channel_config(&ch);

    for (int i = 0; i < LCD_COUNT; i++) { st7735_init_one(i); display_fill(i, 0x0000); }
    ESP_LOGI(TAG, "Displays ready");
}

void display_set_brightness(uint8_t pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (pct * 255) / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void display_fill(int tube, uint16_t color)
{
    if (tube < 0 || tube >= LCD_COUNT) return;
    select_tube(tube);
    lcd_cmd(0x2A); uint8_t ca[] = {0,LCD_OFFSET_X,0,LCD_OFFSET_X+LCD_WIDTH-1}; lcd_data(ca,4);
    lcd_cmd(0x2B); uint8_t ra[] = {0,LCD_OFFSET_Y,0,LCD_OFFSET_Y+LCD_HEIGHT-1}; lcd_data(ra,4);
    lcd_cmd(0x2C);
    gpio_set_level(PIN_LCD_DC, 1);
    uint8_t line[LCD_WIDTH * 2];
    for (int x = 0; x < LCD_WIDTH; x++) { line[x*2] = color>>8; line[x*2+1] = color&0xFF; }
    for (int y = 0; y < LCD_HEIGHT; y++) {
        spi_transaction_t t = { .length = sizeof(line)*8, .tx_buffer = line };
        spi_device_polling_transmit(spi_dev, &t);
    }
    deselect_all();
}

void display_show_digit(int tube, const uint8_t *data, int w, int h)
{
    if (tube < 0 || tube >= LCD_COUNT || !data) return;
    select_tube(tube);
    lcd_cmd(0x2A); uint8_t ca[] = {0,LCD_OFFSET_X,0,LCD_OFFSET_X+w-1}; lcd_data(ca,4);
    lcd_cmd(0x2B); uint8_t ra[] = {0,LCD_OFFSET_Y,0,LCD_OFFSET_Y+h-1}; lcd_data(ra,4);
    lcd_cmd(0x2C);
    gpio_set_level(PIN_LCD_DC, 1);
    spi_transaction_t t = { .length = w*h*2*8, .tx_buffer = data };
    spi_device_polling_transmit(spi_dev, &t);
    deselect_all();
}

void display_show_time(int h, int m, int s, const char *theme)
{
    /* TODO: Load theme digit images from SPIFFS. For now: coloured fills per digit. */
    int digits[6] = {h/10, h%10, m/10, m%10, s/10, s%10};
    for (int i = 0; i < 6; i++) {
        uint16_t r = (digits[i]*3)&0x1F, g = (16+digits[i]*6)&0x3F, b = (31-digits[i]*2)&0x1F;
        display_fill(i, (r<<11)|(g<<5)|b);
    }
}
