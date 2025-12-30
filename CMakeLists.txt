cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(nightfall-fw C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(NIGHTFALL_USER_SOURCES
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
    platform/stm32f405/Core/Src/shortest_run_params.c
    platform/stm32f405/Core/Src/shortest_run_params_split.c
    platform/stm32f405/Core/Src/search_run_params_split.c
    platform/stm32f405/Core/Src/test_mode.c
)

function(nightfall_add_firmware target variant variant_define)
    add_executable(${target})

    target_sources(${target} PRIVATE ${NIGHTFALL_USER_SOURCES})

    set(NIGHTFALL_TURN_BANK_GEN_C ${CMAKE_BINARY_DIR}/generated/${variant}/shortest_turn_bank_gen.c)
    add_custom_command(
        OUTPUT ${NIGHTFALL_TURN_BANK_GEN_C}
        COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/gen_shortest_turn_bank.py
            --csv ${CMAKE_SOURCE_DIR}/params/${variant}/shortest_turn_bank.csv
            --out ${NIGHTFALL_TURN_BANK_GEN_C}
        DEPENDS
            ${CMAKE_SOURCE_DIR}/tools/gen_shortest_turn_bank.py
            ${CMAKE_SOURCE_DIR}/params/${variant}/shortest_turn_bank.csv
        VERBATIM
    )
    set_source_files_properties(${NIGHTFALL_TURN_BANK_GEN_C} PROPERTIES GENERATED TRUE)
    target_sources(${target} PRIVATE ${NIGHTFALL_TURN_BANK_GEN_C})

    set(NIGHTFALL_CASE_PARAMS_GEN_C ${CMAKE_BINARY_DIR}/generated/${variant}/shortest_case_params_gen.c)
    add_custom_command(
        OUTPUT ${NIGHTFALL_CASE_PARAMS_GEN_C}
        COMMAND python3 ${CMAKE_SOURCE_DIR}/tools/gen_shortest_case_params.py
            --csv ${CMAKE_SOURCE_DIR}/params/${variant}/shortest_case_params.csv
            --out ${NIGHTFALL_CASE_PARAMS_GEN_C}
        DEPENDS
            ${CMAKE_SOURCE_DIR}/tools/gen_shortest_case_params.py
            ${CMAKE_SOURCE_DIR}/params/${variant}/shortest_case_params.csv
        VERBATIM
    )
    set_source_files_properties(${NIGHTFALL_CASE_PARAMS_GEN_C} PROPERTIES GENERATED TRUE)
    target_sources(${target} PRIVATE ${NIGHTFALL_CASE_PARAMS_GEN_C})

    nightfall_apply_stm32f405(${target})

    target_include_directories(${target} BEFORE PRIVATE
        ${CMAKE_SOURCE_DIR}/params/${variant}
    )

    target_compile_definitions(${target} PRIVATE
        ${variant_define}
    )

    target_link_options(${target} PRIVATE
        -u _printf_float
        -Wl,-Map=${target}.map
    )

    set_target_properties(${target} PROPERTIES ADDITIONAL_CLEAN_FILES ${target}.map)

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${target}.bin
        COMMENT "Generating binary file for UART flashing: ${target}.bin"
    )

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${target}.hex
        COMMENT "Generating hex file: ${target}.hex"
    )
endfunction()

nightfall_add_firmware(nightfall_mini_v1_1 mini_v1_1 ROBOT_MINI_V1_1)
nightfall_add_firmware(nightfall_classic_v2 classic_v2 ROBOT_CLASSIC_V2)
