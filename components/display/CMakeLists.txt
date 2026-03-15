idf_component_register(
    SRCS "display.c"
    INCLUDE_DIRS "include"
    REQUIRES driver config_mgr board esp_jpeg esp_vfs ntp_time weather youtube_bili
)