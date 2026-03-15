idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        board
        config_mgr
        display
        audio
        leds
        touch
        rtc
        wifi_manager
        web_server
        ntp_time
        weather
        youtube_bili
)
