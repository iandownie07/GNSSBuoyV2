cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME microSWIFT_U5)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(CMAKE_C_FLAGS_DEBUG "-g -O0")  # Enable debugging symbols, disable optimizations

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE CMSIS_DSP_SOURCES
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/*.c
)

file(GLOB PERIPHERAL_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/Peripherals/*.c
)

file(GLOB COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/Common/*.c
)

file(GLOB NEDWAVES_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/NEDWaves/*.c
)

file(GLOB CORE_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/*.c
)

# Remove wrapper files that include other sources internally
list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*/.*Functions\\.c$")
list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*/CommonTables\\.c$")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CORE_SOURCES}
    ${CMSIS_DSP_SOURCES}
    ${PERIPHERAL_SOURCES}
    ${COMMON_SOURCES}
    ${NEDWAVES_SOURCES}
)

list(REMOVE_ITEM PERIPHERAL_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/Peripherals/temp_sensor.c
)

list(REMOVE_ITEM NEDWAVES_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/NEDWaves/interp1.c
)

list(REMOVE_ITEM COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/flash_storage.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM33
    ARM_MATH_FLOAT32
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
