cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "components")

# ── Read firmware version from version.json ──────────────────────────
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.json" _VER_JSON)
string(REGEX MATCH "\"version\"[[:space:]]*:[[:space:]]*\"([^\"]+)\"" _ "${_VER_JSON}")
set(FW_VERSION "${CMAKE_MATCH_1}")
if(NOT FW_VERSION)
    set(FW_VERSION "0.0.0")
endif()

# Write version into the SPIFFS web tree so the UI can detect mismatches at runtime
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/data/web/version.txt" "${FW_VERSION}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Expose version string to all C translation units.
# MUST be before project() – by the time project() runs, all component targets
# are already configured, so any COMPILE_DEFINITIONS set afterwards are ignored.
idf_build_set_property(COMPILE_DEFINITIONS "FW_VERSION_STR=\"${FW_VERSION}\"" APPEND)

project(nextube-fw)

message(STATUS "Nextube-Remaster firmware v${FW_VERSION}")

# Embed the SPIFFS web filesystem into the firmware
spiffs_create_partition_image(spiffs data FLASH_IN_PROJECT)
