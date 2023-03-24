
set(SRC_CLASSES)
set(LVGL_DEMOS_DIR ../managed_components/lvgl__lvgl)
file(GLOB SRC_CLASSES *.c *.cpp)

file(GLOB_RECURSE LV_DEMOS_SRC  ${LVGL_DEMOS_DIR}/demos/*.c ${LVGL_DEMOS_DIR}/src/extra/*.c )

idf_component_register(SRCS ${SRC_CLASSES} ${LV_DEMOS_SRC} 
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver  esp_lcd esp_timer
                       REQUIRES  TouchLib )

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++2b "-Wmissing-field-initializers")