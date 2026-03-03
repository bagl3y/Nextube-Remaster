#include "weather.h"
#include "config_mgr.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "weather";
static weather_data_t s_weather = {0};

static void fetch_weather(void)
{
    const nextube_config_t *cfg = config_get();
    if (strlen(cfg->weather_api_key) == 0 || strlen(cfg->city) == 0) return;

    char url[256];
    snprintf(url, sizeof(url),
        "http://api.openweathermap.org/data/2.5/weather?q=%s&appid=%s&units=metric",
        cfg->city, cfg->weather_api_key);

    esp_http_client_config_t http_cfg = { .url = url, .timeout_ms = 10000 };
    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK && esp_http_client_get_status_code(client) == 200) {
        int len = esp_http_client_get_content_length(client);
        if (len > 0 && len < 2048) {
            char *buf = malloc(len + 1);
            esp_http_client_read(client, buf, len);
            buf[len] = 0;

            cJSON *root = cJSON_Parse(buf);
            if (root) {
                cJSON *main_obj = cJSON_GetObjectItem(root, "main");
                if (main_obj) {
                    cJSON *temp = cJSON_GetObjectItem(main_obj, "temp");
                    cJSON *hum  = cJSON_GetObjectItem(main_obj, "humidity");
                    if (temp) s_weather.temp_c = temp->valuedouble;
                    if (hum)  s_weather.humidity = hum->valuedouble;
                }
                cJSON *weather_arr = cJSON_GetObjectItem(root, "weather");
                if (cJSON_IsArray(weather_arr) && cJSON_GetArraySize(weather_arr) > 0) {
                    cJSON *w0 = cJSON_GetArrayItem(weather_arr, 0);
                    cJSON *desc = cJSON_GetObjectItem(w0, "main");
                    cJSON *icon = cJSON_GetObjectItem(w0, "icon");
                    if (desc) strncpy(s_weather.condition, desc->valuestring, sizeof(s_weather.condition)-1);
                    if (icon) strncpy(s_weather.icon, icon->valuestring, sizeof(s_weather.icon)-1);
                }
                s_weather.valid = true;
                cJSON_Delete(root);
            }
            free(buf);
        }
    } else {
        ESP_LOGW(TAG, "Weather fetch failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

static void weather_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(15000));  /* Wait for WiFi */
    while (1) {
        fetch_weather();
        vTaskDelay(pdMS_TO_TICKS(600000));  /* Every 10 minutes */
    }
}

void weather_start(void) { xTaskCreate(weather_task, "weather", 8192, NULL, 3, NULL); }
const weather_data_t *weather_get(void) { return &s_weather; }
