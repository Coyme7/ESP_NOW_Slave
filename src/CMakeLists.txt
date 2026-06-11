# 从机 src 组件清单。
# 只收集可编译源文件，避免把头文件、压缩包或说明文件传给 idf_component_register。
file(GLOB_RECURSE app_sources
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.S"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "../include"
    REQUIRES framework-arduinoespressif32 esp_wifi esp_timer esp_adc
)
