cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app)

target_sources(app PRIVATE src/main.c
                    src/functionalities.c
                    src/functionalities.h
                    src/project.h
                    src/u_error_common.h
                    src/u_ubx_protocol.c
                    src/u_ubx_protocol.h)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/boards>)
# list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ubx )