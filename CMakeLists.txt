cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components")

# ── Read firmware version from version.json ──────────────────────────
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.json" _VER_JSON)
string(REGEX MATCH "\"version\"[[:space:]]*:[[:space:]]*\"([^\"]+)\"" _ "${_VER_JSON}")
set(FW_VERSION "${CMAKE_MATCH_1}")
if(NOT FW_VERSION)
    set(FW_VERSION "0.0.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(nextube-fw)

message(STATUS "Nextube-Remaster firmware v${FW_VERSION}")

# Expose version string to all C translation units
idf_build_set_property(COMPILE_DEFINITIONS "-DFW_VERSION_STR=\"${FW_VERSION}\"" APPEND)

# Embed the SPIFFS web filesystem into the firmware
spiffs_create_partition_image(spiffs data FLASH_IN_PROJECT)
