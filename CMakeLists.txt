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

# Firmware version - update this when releasing a new version
set(FW_VERSION "1.0.0")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(NIGHTFALL_STM32F405_USER_SOURCES
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
    platform/stm32f405/Core/Src/test_mode.c
)

function(nightfall_apply_mcu target mcu_family)
    if(mcu_family STREQUAL "stm32f405")
        nightfall_apply_stm32f405(${target})
    elseif(mcu_family STREQUAL "stm32f413")
        nightfall_apply_stm32f413(${target})
    else()
        message(FATAL_ERROR "Unsupported MCU family: ${mcu_family}")
    endif()
endfunction()

function(nightfall_add_common_post_build_steps target)
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

function(nightfall_add_firmware target variant variant_define mcu_family)
    add_executable(${target})

    set(build_info_dir ${CMAKE_BINARY_DIR}/generated/${target})
    set(build_info_h ${build_info_dir}/build_info.h)

    set(_src_deps)
    foreach(_s IN LISTS NIGHTFALL_STM32F405_USER_SOURCES)
        list(APPEND _src_deps "${CMAKE_SOURCE_DIR}/${_s}")
    endforeach()
    list(APPEND _src_deps "${CMAKE_SOURCE_DIR}/params/${variant}/params.h")

    set(_variant_src
        "${CMAKE_SOURCE_DIR}/params/${variant}/shortest_run_params_split.c"
        "${CMAKE_SOURCE_DIR}/params/${variant}/search_run_params_split.c"
    )
    list(APPEND _src_deps ${_variant_src})

    set(_git_deps)
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/HEAD")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/HEAD")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/index")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/index")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/packed-refs")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/packed-refs")
    endif()

    add_custom_command(
        OUTPUT ${build_info_h}
        COMMAND ${CMAKE_COMMAND}
            -DOUTPUT_FILE=${build_info_h}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DFW_TARGET=${target}
            -DFW_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DFW_VERSION=${FW_VERSION}
            -P ${CMAKE_SOURCE_DIR}/cmake/gen_build_info.cmake
        DEPENDS ${CMAKE_SOURCE_DIR}/cmake/gen_build_info.cmake ${_git_deps} ${_src_deps}
        VERBATIM
        COMMENT "Generating build_info.h for ${target}"
    )

    add_custom_target(build_info_${target} DEPENDS ${build_info_h})
    add_dependencies(${target} build_info_${target})

    target_sources(${target} PRIVATE ${NIGHTFALL_STM32F405_USER_SOURCES} ${_variant_src})

    nightfall_apply_mcu(${target} ${mcu_family})

    target_include_directories(${target} BEFORE PRIVATE
        ${CMAKE_SOURCE_DIR}/params/${variant}
        ${build_info_dir}
    )

    target_compile_definitions(${target} PRIVATE
        ${variant_define}
    )

    nightfall_add_common_post_build_steps(${target})
endfunction()

function(nightfall_add_platform_target target mcu_family)
    add_executable(${target})

    set(build_info_dir ${CMAKE_BINARY_DIR}/generated/${target})
    set(build_info_h ${build_info_dir}/build_info.h)

    set(_git_deps)
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/HEAD")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/HEAD")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/index")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/index")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/.git/packed-refs")
        list(APPEND _git_deps "${CMAKE_SOURCE_DIR}/.git/packed-refs")
    endif()

    add_custom_command(
        OUTPUT ${build_info_h}
        COMMAND ${CMAKE_COMMAND}
            -DOUTPUT_FILE=${build_info_h}
            -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
            -DFW_TARGET=${target}
            -DFW_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DFW_VERSION=${FW_VERSION}
            -P ${CMAKE_SOURCE_DIR}/cmake/gen_build_info.cmake
        DEPENDS ${CMAKE_SOURCE_DIR}/cmake/gen_build_info.cmake ${_git_deps}
        VERBATIM
        COMMENT "Generating build_info.h for ${target}"
    )

    add_custom_target(build_info_${target} DEPENDS ${build_info_h})
    add_dependencies(${target} build_info_${target})

    nightfall_apply_mcu(${target} ${mcu_family})

    target_include_directories(${target} BEFORE PRIVATE
        ${build_info_dir}
    )

    nightfall_add_common_post_build_steps(${target})
endfunction()

nightfall_add_firmware(nightfall_mini_r1_0 mini_r1_0 ROBOT_MINI_R1_0 stm32f405)
nightfall_add_firmware(nightfall_classic_r1_0 classic_r1_0 ROBOT_CLASSIC_R1_0 stm32f405)
nightfall_add_platform_target(nightfall_stm32f413 stm32f413)

add_custom_target(nightfall_stm32f405
    DEPENDS nightfall_mini_r1_0 nightfall_classic_r1_0
)
