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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME LSM6DSO)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add user sources
file(GLOB SSD1306_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/ssd1306/*.c"
)

file(GLOB IMU_LSM6DSO_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/IMU_LSM6DSO/*.c"
)

file(GLOB YAW_ESTIMATOR_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/Yaw_Estimator/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${SSD1306_SOURCES}
    ${IMU_LSM6DSO_SOURCES}
    ${YAW_ESTIMATOR_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Drivers/ssd1306
    ${CMAKE_SOURCE_DIR}/Drivers/IMU_LSM6DSO
    ${CMAKE_SOURCE_DIR}/Drivers/Yaw_Estimator
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Compile Options: Force usage of FPU registers (Hard Float)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Link Options: Link against Hard Float libraries and enable float printf
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float        # Keeps your existing float printing support
    -Wl,--print-memory-usage # Optional: Prints RAM/Flash usage after build
)