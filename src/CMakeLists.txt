# 从机 src 组件清单。
# PlatformIO + ESP-IDF 会从这里收集 src 下的所有源码并注册为应用组件。
# 这里不要写主从共享构建逻辑；主机和从机必须继续作为两个独立工程编译、烧录。
# REQUIRES 中显式列出 Arduino 组件和 esp_wifi，是因为当前 ESP-NOW 与串口入口依赖它们。

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "../include"
    REQUIRES framework-arduinoespressif32 esp_wifi
)
