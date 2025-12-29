cmake_minimum_required(VERSION 3.22)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
    USE_HAL_DRIVER
    STM32F405xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Inc
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/stm32f4xx_it.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/startup_stm32f405xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Link directories setup
set(MX_LINK_DIRS
)

# Project static libraries
set(MX_LINK_LIBS
    STM32_Drivers
)

set(NIGHTFALL_MX_APPLICATION_SRC ${MX_Application_Src} PARENT_SCOPE)
set(NIGHTFALL_MX_LINK_DIRS ${MX_LINK_DIRS} PARENT_SCOPE)
set(NIGHTFALL_MX_LINK_LIBS ${MX_LINK_LIBS} PARENT_SCOPE)

# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)

# Suppress benign warnings from HAL drivers
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(STM32_Drivers PRIVATE -Wno-unused-parameter)
endif()

function(nightfall_apply_stm32f405 target)
    # Add STM32CubeMX generated application sources to the target
    target_sources(${target} PRIVATE ${NIGHTFALL_MX_APPLICATION_SRC})

    # Link directories setup
    target_link_directories(${target} PRIVATE ${NIGHTFALL_MX_LINK_DIRS})

    # Add libraries to the target
    target_link_libraries(${target} stm32cubemx ${NIGHTFALL_MX_LINK_LIBS})
endfunction()

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
