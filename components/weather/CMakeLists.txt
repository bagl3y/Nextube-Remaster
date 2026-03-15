idf_component_register(
    SRCS "weather.c"
    INCLUDE_DIRS "include"
    REQUIRES config_mgr esp_http_client esp_crt_bundle json
)