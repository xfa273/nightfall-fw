cmake_minimum_required(VERSION 3.22)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

set(NIGHTFALL_STM32_COMMON_LINK_OPTIONS
    --specs=nano.specs
    -Wl,--gc-sections
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,--print-memory-usage
)
set(NIGHTFALL_STM32_COMMON_LINK_OPTIONS ${NIGHTFALL_STM32_COMMON_LINK_OPTIONS} PARENT_SCOPE)

set(NIGHTFALL_STM32F405_ROOT ${CMAKE_SOURCE_DIR}/platform/stm32f405)
set(NIGHTFALL_STM32F405_LINKER_SCRIPT ${NIGHTFALL_STM32F405_ROOT}/STM32F405XX_FLASH.ld)
set(NIGHTFALL_STM32F405_APPLICATION_SRC
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/main.c
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/stm32f4xx_it.c
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/stm32f4xx_hal_msp.c
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/sysmem.c
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/syscalls.c
    ${NIGHTFALL_STM32F405_ROOT}/startup_stm32f405xx.s
)
set(NIGHTFALL_STM32F405_LINKER_SCRIPT ${NIGHTFALL_STM32F405_LINKER_SCRIPT} PARENT_SCOPE)
set(NIGHTFALL_STM32F405_APPLICATION_SRC ${NIGHTFALL_STM32F405_APPLICATION_SRC} PARENT_SCOPE)
set(NIGHTFALL_STM32F405_DRIVERS_SRC
    ${NIGHTFALL_STM32F405_ROOT}/Core/Src/system_stm32f4xx.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

add_library(stm32cubemx_stm32f405 INTERFACE)
target_include_directories(stm32cubemx_stm32f405 INTERFACE
    ${CMAKE_SOURCE_DIR}/nvm
    ${CMAKE_SOURCE_DIR}/platform/trace
    ${NIGHTFALL_STM32F405_ROOT}/Core/Inc
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${NIGHTFALL_STM32F405_ROOT}/Drivers/CMSIS/Include
)
target_compile_definitions(stm32cubemx_stm32f405 INTERFACE
    USE_HAL_DRIVER
    STM32F405xx
    $<$<CONFIG:Debug>:DEBUG>
)

add_library(stm32_drivers_stm32f405 OBJECT)
target_sources(stm32_drivers_stm32f405 PRIVATE ${NIGHTFALL_STM32F405_DRIVERS_SRC})
target_link_libraries(stm32_drivers_stm32f405 PUBLIC stm32cubemx_stm32f405)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stm32_drivers_stm32f405 PRIVATE -Wno-unused-parameter)
endif()

set(NIGHTFALL_STM32F413_ROOT ${CMAKE_SOURCE_DIR}/platform/stm32f413/HM_Nightfall_f413_preorder)
set(NIGHTFALL_STM32F413_LINKER_SCRIPT ${NIGHTFALL_STM32F413_ROOT}/STM32F413XX_FLASH.ld)
set(NIGHTFALL_TRACE_F413_USE_UART "1" CACHE STRING "Use USART1 trace output on STM32F413 instead of SWO/ITM")
set_property(CACHE NIGHTFALL_TRACE_F413_USE_UART PROPERTY STRINGS "0" "1")
set(NIGHTFALL_F413_UART_BAUD_RATE "921600" CACHE STRING "USART1 baud rate for STM32F413 UART command and trace output")
set_property(CACHE NIGHTFALL_F413_UART_BAUD_RATE PROPERTY STRINGS "115200" "460800" "921600" "1000000")
set(NIGHTFALL_STM32F413_APPLICATION_SRC
    ${CMAKE_SOURCE_DIR}/nvm/nvm.c
    ${CMAKE_SOURCE_DIR}/nvm/nvm_identity.c
    ${CMAKE_SOURCE_DIR}/nvm/nvm_params.c
    ${CMAKE_SOURCE_DIR}/nvm/nvm_trace_log.c
    ${CMAKE_SOURCE_DIR}/platform/trace/trace.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/sensor_distance.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/solver.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/solver_params.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/path.c
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Src/maze_grid.c
    ${CMAKE_SOURCE_DIR}/params/f413_preorder/shortest_run_params_split.c
    ${CMAKE_SOURCE_DIR}/params/f413_preorder/search_run_params_split.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_solver_bridge.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_control.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_diag.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_hw.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_hw_diag.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_imu_diag.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_op_ui.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_run_session.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_search_step.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_test_run.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_trace_diag.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_trace_log.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/f413_wall_sensor.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/main.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/stm32f4xx_it.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/stm32f4xx_hal_msp.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/sysmem.c
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/syscalls.c
    ${NIGHTFALL_STM32F413_ROOT}/startup_stm32f413xx.s
)
set(NIGHTFALL_STM32F413_LINKER_SCRIPT ${NIGHTFALL_STM32F413_LINKER_SCRIPT} PARENT_SCOPE)
set(NIGHTFALL_STM32F413_APPLICATION_SRC ${NIGHTFALL_STM32F413_APPLICATION_SRC} PARENT_SCOPE)
set(NIGHTFALL_STM32F413_DRIVERS_SRC
    ${NIGHTFALL_STM32F413_ROOT}/Core/Src/system_stm32f4xx.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

add_library(stm32cubemx_stm32f413 INTERFACE)
target_include_directories(stm32cubemx_stm32f413 INTERFACE
    ${CMAKE_SOURCE_DIR}/nvm
    ${CMAKE_SOURCE_DIR}/params/f413_preorder
    ${CMAKE_SOURCE_DIR}/platform/trace
    ${NIGHTFALL_STM32F413_ROOT}/Core/Inc
    ${CMAKE_SOURCE_DIR}/platform/stm32f405/Core/Inc
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${NIGHTFALL_STM32F413_ROOT}/Drivers/CMSIS/Include
)
target_compile_definitions(stm32cubemx_stm32f413 INTERFACE
    USE_HAL_DRIVER
    STM32F413xx
    NIGHTFALL_TRACE_F413_USE_UART=${NIGHTFALL_TRACE_F413_USE_UART}
    NIGHTFALL_F413_UART_BAUD_RATE=${NIGHTFALL_F413_UART_BAUD_RATE}
    NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1
    $<$<CONFIG:Debug>:DEBUG>
)

add_library(stm32_drivers_stm32f413 OBJECT)
target_sources(stm32_drivers_stm32f413 PRIVATE ${NIGHTFALL_STM32F413_DRIVERS_SRC})
target_link_libraries(stm32_drivers_stm32f413 PUBLIC stm32cubemx_stm32f413)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stm32_drivers_stm32f413 PRIVATE -Wno-unused-parameter)
endif()

function(nightfall_apply_stm32f405 target)
    target_sources(${target} PRIVATE ${NIGHTFALL_STM32F405_APPLICATION_SRC})
    target_link_libraries(${target} PRIVATE stm32cubemx_stm32f405 stm32_drivers_stm32f405)
    target_link_options(${target} PRIVATE
        -T${NIGHTFALL_STM32F405_LINKER_SCRIPT}
        ${NIGHTFALL_STM32_COMMON_LINK_OPTIONS}
    )
endfunction()

function(nightfall_apply_stm32f413 target)
    target_sources(${target} PRIVATE ${NIGHTFALL_STM32F413_APPLICATION_SRC})
    target_link_libraries(${target} PRIVATE stm32cubemx_stm32f413 stm32_drivers_stm32f413)
    target_link_options(${target} PRIVATE
        -T${NIGHTFALL_STM32F413_LINKER_SCRIPT}
        ${NIGHTFALL_STM32_COMMON_LINK_OPTIONS}
    )
endfunction()

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
