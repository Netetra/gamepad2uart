# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gamepad2uart C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(gamepad2uart main.cpp ./include/hid_report_parser.cpp)

pico_set_program_name(gamepad2uart "gamepad2uart")
pico_set_program_version(gamepad2uart "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(gamepad2uart 1)
pico_enable_stdio_usb(gamepad2uart 0)

# Add the standard library to the build
target_link_libraries(gamepad2uart
    pico_stdlib
    pico_multicore
    tinyusb_board
    tinyusb_host
)

# Add the standard include files to the build
target_include_directories(gamepad2uart PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_include_directories(gamepad2uart PRIVATE ./include)

target_compile_options(gamepad2uart PRIVATE
    -Wno-narrowing
    -Wno-shift-count-overflow
)

pico_add_extra_outputs(gamepad2uart)

