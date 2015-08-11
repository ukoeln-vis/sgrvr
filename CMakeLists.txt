cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake" 
)


set(COMMON_DIR "${PROJECT_SOURCE_DIR}/src/common")
set(CMD_LINE_DIR "${COMMON_DIR}/CmdLine")

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${CMD_LINE_DIR}/include")

add_subdirectory(src)
