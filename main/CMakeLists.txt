idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        config_mgr
        display
        leds
        touch
        rtc
        wifi_manager
        web_server
        ntp_time
        weather
        youtube_bili
)
