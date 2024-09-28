# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")
#set(PICO_BOARD pico_2????????????????? CACHE STRING "Board type")
#set(WIFI_SSID "Your Network")
#set(WIFI_PASSWORD "Your Password")
#set(TEST_TCP_SERVER_IP "192.168.0.1")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(new_pio C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(new_pio new_pio.c )

pico_set_program_name(new_pio "new_pio")
pico_set_program_version(new_pio "0.1")

# Generate PIO header
pico_generate_pio_header(new_pio ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(new_pio 1)
pico_enable_stdio_usb(new_pio 1) # make this a 1 to turn on the usb - must do for uart usb comm

# Add the standard library to the build
target_link_libraries(new_pio
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(new_pio PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(new_pio 
        hardware_pio
        hardware_clocks
        )

pico_add_extra_outputs(new_pio)

