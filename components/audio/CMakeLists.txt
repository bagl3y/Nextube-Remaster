idf_component_register(
    SRCS "audio.c"
    INCLUDE_DIRS "include"
    REQUIRES driver config_mgr board leds
    PRIV_REQUIRES esp_timer
)