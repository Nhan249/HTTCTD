#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "driver/i2c.h"

#include "esp_http_server.h"

// =====================================================
//                  CẤU HÌNH HỆ THỐNG
// =====================================================
#define WIFI_SSID      "Ming"
#define WIFI_PASS      "Ming2003"

#define SOIL_ADC_UNIT      ADC_UNIT_1
#define SOIL_ADC_CHANNEL   ADC_CHANNEL_0  // GPIO36
#define SOIL_ADC_ATTEN     ADC_ATTEN_DB_12
#define SOIL_ADC_BITWIDTH  ADC_BITWIDTH_DEFAULT

#define SOIL_ADC_DRY       3500  // Giá trị mV đất khô
#define SOIL_ADC_WET       1425  // Giá trị mV đất ướt

#define MOISTURE_LOW_THRESHOLD  40
#define MOISTURE_HIGH_THRESHOLD 50

#define PUMP_GPIO         GPIO_NUM_23

#define I2C_MASTER_SCL   22
#define I2C_MASTER_SDA   21
#define I2C_MASTER_NUM   I2C_NUM_0
#define I2C_FREQ_HZ      100000

static const char *TAG = "TREE_PROJECT";

// =====================================================
//                        WIFI
// =====================================================
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Hàm callback xử lý sự kiện WiFi
// --------------------------------------
// Khi ESP32 bắt đầu: tự động kết nối
// Khi mất WiFi → tự kết nối lại
// Khi có IP kết nối thành công
//---------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi mất kết nối, đang thử kết nối lại...");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Đã kết nối Wi-Fi, IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// Hàm khởi tạo WiFi STA
// ------------------------
// Khởi tạo NVS
// Khởi tạo WiFi driver
// Đăng ký event handler
// Kết nối vào WiFi cấu hình sẵn
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA đã khởi động");
}

// =====================================================
//                        ADC
// =====================================================
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t soil_adc_cali_handle = NULL;

// Hàm khởi tạo ADC One-shot
// ---------------------------
// Tạo unit ADC
// Cấu hình kênh được chọn
// Bật chế độ calibration nếu có
void soil_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = SOIL_ADC_UNIT};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = SOIL_ADC_ATTEN, .bitwidth = SOIL_ADC_BITWIDTH};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_ADC_CHANNEL, &chan_cfg));

    adc_cali_line_fitting_config_t cal_cfg = {.unit_id = SOIL_ADC_UNIT,
                                              .atten = SOIL_ADC_ATTEN,
                                              .bitwidth = SOIL_ADC_BITWIDTH};
    if (adc_cali_create_scheme_line_fitting(&cal_cfg, &soil_adc_cali_handle) != ESP_OK) {
        ESP_LOGW(TAG, "Không hỗ trợ calibration, dùng raw ADC");
        soil_adc_cali_handle = NULL;
    }
}

// Đọc ADC trả về mili-volt
// --------------------------------
uint32_t read_soil_moisture_mV(void)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, SOIL_ADC_CHANNEL, &raw));

    int voltage = 0;
    if (soil_adc_cali_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(soil_adc_cali_handle, raw, &voltage));
    } else {
        voltage = raw * 3300 / 4095;
    }
    return (uint32_t)voltage;
}

// Chuyển đổi mV → % độ ẩm
// -----------------------------------
// percent = (khô - mv) * 100 / (khô - ướt)
int soil_moisture_percent(uint32_t mV)
{
    int percent = (SOIL_ADC_DRY - mV) * 100 / (SOIL_ADC_DRY - SOIL_ADC_WET);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

// =====================================================
//                        LCD I2C
// =====================================================

// Gửi 1 byte tới PCF8574 qua I2C
esp_err_t lcd_write(uint8_t data)
{
    uint8_t buf = data;
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x27, &buf, 1, 1000 / portTICK_PERIOD_MS);
}

// Gửi lệnh đến LCD
void lcd_cmd(uint8_t cmd)
{
    uint8_t high = (cmd & 0xF0);
    uint8_t low  = ((cmd << 4) & 0xF0);

    lcd_write(high | 0x0C);
    lcd_write(high | 0x08);
    lcd_write(low | 0x0C);
    lcd_write(low | 0x08);
}

// Gửi ký tự
void lcd_data(uint8_t data)
{
    uint8_t high = (data & 0xF0);
    uint8_t low  = ((data << 4) & 0xF0);

    lcd_write(high | 0x0D);
    lcd_write(high | 0x09);
    lcd_write(low | 0x0D);
    lcd_write(low | 0x09);
}

// Ghi chuỗi ra LCD
void lcd_print(const char *str)
{
    while (*str) lcd_data(*str++);
}

// Đặt con trỏ LCD
void lcd_set_cursor(int row, int col)
{
    uint8_t pos = (row == 0 ? 0x80 : 0xC0) + col;
    lcd_cmd(pos);
}

// Khởi tạo LCD
void lcd_init()
{
    vTaskDelay(50 / portTICK_PERIOD_MS);
    lcd_cmd(0x33);
    lcd_cmd(0x32);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

// =====================================================
//                        RELAY
// =====================================================
volatile int g_soil_percent = 0;
volatile bool g_pump_status = false;
volatile bool g_auto_mode = true;

void relay_init(void)
{
    gpio_reset_pin(PUMP_GPIO);
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_GPIO, 0);
}

// Bật/tắt bơm
void pump_control(bool on)
{
    g_pump_status = on;
    gpio_set_level(PUMP_GPIO, on ? 1 : 0);
    ESP_LOGI(TAG, "Pump %s", on ? "ON" : "OFF");
}

// =====================================================
//                        TASKS
// =====================================================

// Task đọc ADC và cập nhật LCD
void soil_task(void *arg)
{
    char buf[16];
    while(1) {
        uint32_t soil_mV = read_soil_moisture_mV();
        g_soil_percent = soil_moisture_percent(soil_mV);

        ESP_LOGI(TAG, "Soil: %lu mV | %d%%", soil_mV, g_soil_percent);

        lcd_set_cursor(0, 0);
        sprintf(buf, "Do am: %3d%%", g_soil_percent);
        lcd_print(buf);

        lcd_set_cursor(1, 0);
        lcd_print(g_pump_status ? "Bom: ON " : "Bom: OFF");

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}


// Task tự động điều khiển bơm
void pump_task(void *arg)
{
    while(1) {
        if (g_auto_mode) {
            if (!g_pump_status && g_soil_percent < MOISTURE_LOW_THRESHOLD)
                pump_control(true);
            else if (g_pump_status && g_soil_percent > MOISTURE_HIGH_THRESHOLD)
                pump_control(false);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// =====================================================
//                    HTTP SERVER API
// =====================================================
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

// Trả về trang HTML
esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    size_t html_size = index_html_end - index_html_start;
    httpd_resp_send(req, index_html_start, html_size);
    return ESP_OK;
}

// API trả về dữ liệu cảm biến
esp_err_t sensor_data_handler(httpd_req_t *req)
{
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"soilMoisture\":%.1f,\"pumpStatus\":%s,\"autoMode\":%s}",
             (float)g_soil_percent,
             g_pump_status ? "true" : "false",
             g_auto_mode ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

// API bật/tắt bơm thủ công
esp_err_t control_pump_handler(httpd_req_t *req)
{
    char content[64];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret > 0) {
        if (strstr(content, "true")) pump_control(true);
        else pump_control(false);
    }
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

// API bật/tắt chế độ AUTO
esp_err_t auto_mode_handler(httpd_req_t *req)
{
    char content[64];
    int ret = httpd_req_recv(req, content, sizeof(content));
    if (ret > 0) {
        g_auto_mode = strstr(content, "true") ? true : false;
    }
    httpd_resp_sendstr(req, "{\"success\":true}");
    return ESP_OK;
}

// =====================================================
//                    KHỞI ĐỘNG WEB SERVER
// =====================================================
httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler
};

httpd_uri_t sensor_data_uri = {
    .uri = "/api/sensor-data",
    .method = HTTP_GET,
    .handler = sensor_data_handler
};

httpd_uri_t control_pump_uri = {
    .uri = "/api/control-pump",
    .method = HTTP_POST,
    .handler = control_pump_handler
};

httpd_uri_t auto_mode_uri = {
    .uri = "/api/auto-mode",
    .method = HTTP_POST,
    .handler = auto_mode_handler
};

void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &sensor_data_uri);
    httpd_register_uri_handler(server, &control_pump_uri);
    httpd_register_uri_handler(server, &auto_mode_uri);

    ESP_LOGI(TAG, "HTTP server started");
}

// =====================================================
//                        MAIN
// =====================================================
void app_main(void)
{
    // WiFi
    wifi_init_sta();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(10000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "Không có WiFi, chạy offline");
    }

    // Khởi tạo ADC và Relay
    soil_adc_init();
    relay_init();

    // Khởi tạo LCD I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    lcd_init();

    // Tạo task
    xTaskCreate(soil_task, "soil_task", 4096, NULL, 5, NULL);
    xTaskCreate(pump_task, "pump_task", 2048, NULL, 5, NULL);

    // Chạy web server
    start_webserver();
}
