# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
  set(USERHOME $ENV{USERPROFILE})
else()
  set(USERHOME $ENV{HOME})
endif()

set(PICO_SDK_PATH ${USERHOME}/.pico-sdk/sdk/1.5.1)
set(PICO_TOOLCHAIN_PATH ${USERHOME}/.pico-sdk/toolchain/13_2_Rel1)

if(WIN32)
  set(pico-sdk-tools_DIR ${USERHOME}/.pico-sdk/tools/1.5.1)
  include(${pico-sdk-tools_DIR}/pico-sdk-tools-config.cmake)
  include(${pico-sdk-tools_DIR}/pico-sdk-tools-config-version.cmake)
endif()

# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if(PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(servo-bench C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(servo-bench servo-bench.cpp)

pico_set_program_name(servo-bench "servo-bench")
pico_set_program_version(servo-bench "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(servo-bench 0)
pico_enable_stdio_usb(servo-bench 1)

# Add the standard library to the build
add_subdirectory(pico-ssd1306)
target_link_libraries(servo-bench
  pico_ssd1306
  pico_stdlib
  hardware_i2c)

# Add the standard include files to the build
target_include_directories(servo-bench PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

pico_add_extra_outputs(servo-bench)
