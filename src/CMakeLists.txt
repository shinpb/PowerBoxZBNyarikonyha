# Component of the firmware: every .c/.cpp file in this directory.

FILE(GLOB_RECURSE app_sources "*.cpp" "*.c")

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_driver_tsens esp_timer
)
