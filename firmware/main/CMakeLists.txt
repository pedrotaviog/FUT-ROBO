idf_build_get_property(target IDF_TARGET)
set(priv_requires "")

if(${target} STREQUAL "linux")
    list(APPEND priv_requires nvs_flash esp_event esp_netif lwip)
endif()

idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES ${priv_requires} nvs_flash esp_event esp_netif lwip driver esp_driver_ledc esp_driver_gpio esp_timer
    INCLUDE_DIRS "." "../../common/include"
)
