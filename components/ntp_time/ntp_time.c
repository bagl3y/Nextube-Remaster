#include "ntp_time.h"
#include "config_mgr.h"
#include "rtc_pcf8563.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "ntp";
static bool s_synced = false;

static void time_sync_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP time synchronised");
    s_synced = true;

    /* Write the freshly-synchronised time back to the battery-backed RTC so
     * it survives power cuts and acts as a warm seed on the next boot.
     * Store local time so mktime() can reconstruct time_t on boot without
     * needing extra UTC handling (TZ is always applied before both writes
     * and reads). */
    struct tm t;
    time_t now = time(NULL);
    localtime_r(&now, &t);
    if (rtc_set_time(&t)) {
        ESP_LOGI(TAG, "RTC updated: %04d-%02d-%02d %02d:%02d:%02d (local)",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        ESP_LOGW(TAG, "RTC write failed after NTP sync");
    }
}

static void ntp_task(void *arg)
{
    const nextube_config_t *cfg = config_get();

    /* Wait for WiFi */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Set timezone */
    char tz[32];
    int hrs = cfg->time_zone / 3600;
    int mins = abs((cfg->time_zone % 3600) / 60);
    snprintf(tz, sizeof(tz), "UTC%+d:%02d", -hrs, mins);
    setenv("TZ", tz, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone set to %s (offset=%ld)", tz, (long)cfg->time_zone);

    /* Seed the system clock from the battery-backed RTC so the display shows
     * a reasonable time immediately, before the first NTP sync completes.
     * rtc_get_time() returns local time; mktime() interprets it as local
     * (TZ is already set above), producing a correct time_t. */
    struct tm rtc_t = {0};
    if (rtc_get_time(&rtc_t)) {
        time_t seed = mktime(&rtc_t);
        if (seed > 0) {
            struct timeval tv_seed = { .tv_sec = seed, .tv_usec = 0 };
            settimeofday(&tv_seed, NULL);
            ESP_LOGI(TAG, "System clock seeded from RTC: %04d-%02d-%02d %02d:%02d:%02d",
                     rtc_t.tm_year + 1900, rtc_t.tm_mon + 1, rtc_t.tm_mday,
                     rtc_t.tm_hour, rtc_t.tm_min, rtc_t.tm_sec);
        } else {
            ESP_LOGW(TAG, "RTC returned invalid time (seed=%lld) – ignoring", (long long)seed);
        }
    } else {
        ESP_LOGW(TAG, "RTC read failed – clock starts at epoch until NTP sync");
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, cfg->ntp_server);
    sntp_set_time_sync_notification_cb(time_sync_cb);
    esp_sntp_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void ntp_apply_timezone(void)
{
    const nextube_config_t *cfg = config_get();
    int hrs  = cfg->time_zone / 3600;
    int mins = abs((cfg->time_zone % 3600) / 60);
    char tz[32];
    snprintf(tz, sizeof(tz), "UTC%+d:%02d", -hrs, mins);
    setenv("TZ", tz, 1);
    tzset();
    ESP_LOGI(TAG, "Timezone updated to %s (offset=%ld s)", tz, (long)cfg->time_zone);
}

void ntp_time_start(void)
{
    xTaskCreate(ntp_task, "ntp", 4096, NULL, 5, NULL);
}

bool ntp_time_synced(void) { return s_synced; }

void ntp_get_local(struct tm *t)
{
    time_t now;
    time(&now);
    localtime_r(&now, t);
}
