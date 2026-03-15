idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "include"
    REQUIRES driver config_mgr board espressif__esp_jpeg ntp_time weather youtube_bili
)