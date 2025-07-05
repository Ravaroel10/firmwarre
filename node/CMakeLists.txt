cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS node master) # atau ganti dengan main_master kalau mau itu

project(wasp_firmware)
