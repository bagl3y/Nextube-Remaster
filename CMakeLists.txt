cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Embed the SPIFFS web filesystem into the firmware
spiffs_create_partition_image(spiffs data FLASH_IN_PROJECT)

project(nextube-fw)
