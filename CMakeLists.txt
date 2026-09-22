cmake_minimum_required(VERSION 3.16)

# This project is intentionally fixed to the Waveshare ESP32-S3-Zero target.
set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rx7500_logic_analyzer)

