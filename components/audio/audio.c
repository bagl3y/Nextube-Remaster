#include "audio.h"
#include "board_pins.h"
#include "esp_log.h"
#include "driver/dac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "audio";
static int s_volume = 20;

void audio_init(void)
{
    ESP_LOGI(TAG, "Initialising DAC audio on GPIO%d", PIN_AUDIO_DAC);
    dac_output_enable(DAC_CHANNEL_1);
}

void audio_play_file(const char *path)
{
    ESP_LOGI(TAG, "Playing: %s (vol=%d)", path, s_volume);
    /* TODO: Implement WAV file playback via DAC DMA.
       Original firmware used AudioGeneratorWAV from ESP8266Audio library. */
}

void audio_set_volume(int vol)
{
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;
    s_volume = vol;
}

void audio_stop(void)
{
    dac_output_voltage(DAC_CHANNEL_1, 128);  /* Silence = midpoint */
}
