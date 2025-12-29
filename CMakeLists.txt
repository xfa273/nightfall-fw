cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME nightfall_mini_v1_1)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    platform/stm32f405/Core/Src/auxiliary.c
    platform/stm32f405/Core/Src/control.c
    platform/stm32f405/Core/Src/distance_params.c
    platform/stm32f405/Core/Src/drive.c
    platform/stm32f405/Core/Src/flash_params.c
    platform/stm32f405/Core/Src/eeprom.c
    platform/stm32f405/Core/Src/interrupt.c
    platform/stm32f405/Core/Src/logging.c
    platform/stm32f405/Core/Src/mode1.c
    platform/stm32f405/Core/Src/mode2.c
    platform/stm32f405/Core/Src/mode3.c
    platform/stm32f405/Core/Src/mode4.c
    platform/stm32f405/Core/Src/mode5.c
    platform/stm32f405/Core/Src/mode6.c
    platform/stm32f405/Core/Src/mode7.c
    platform/stm32f405/Core/Src/maze_grid.c
    platform/stm32f405/Core/Src/solver_params.c
    platform/stm32f405/Core/Src/solver.c
    platform/stm32f405/Core/Src/path.c
    platform/stm32f405/Core/Src/run.c
    platform/stm32f405/Core/Src/search.c
    platform/stm32f405/Core/Src/sensor.c
    platform/stm32f405/Core/Src/sensor_distance.c
    platform/stm32f405/Core/Src/shortest_run_params_split.c
    platform/stm32f405/Core/Src/search_run_params_split.c
    platform/stm32f405/Core/Src/test_mode.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# Enable printf float support
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Generate .bin file for UART flashing with CubeProgrammer
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating binary file for UART flashing: ${CMAKE_PROJECT_NAME}.bin"
)

# Generate .hex file (optional)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating hex file: ${CMAKE_PROJECT_NAME}.hex"
)
