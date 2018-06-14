cmake_minimum_required(VERSION 3.6)
set(PRJ_NAME sali)
include(STM32Toolchain.cmake)

set(MCU_FAMILY STM32F1xx)
set(MCU_LINE STM32F103xx)
set(MCU_TARGET STM32F103c8t6)
set(FREERTOS_DIR Middlewares/Third_Party/FreeRTOS/Source)
project(${PRJ_NAME} C CXX ASM)

add_definitions(-D${MCU_LINE})
add_definitions(-DUSE_HAL_LIBRARY)

file(GLOB_RECURSE USER_SOURCES src/*.c)

file(GLOB FREERTOS_SOURCES ${FREERTOS_DIR}/*.c)
file(GLOB CMSIS_RTOS ${FREERTOS_DIR}/CMSIS_RTOS/*.c)
file(GLOB FREERTOS_SOURCES_MEM ${FREERTOS_DIR}/portable/MemMang/heap_4.c)
file(GLOB FREERTOS_SOURCES_PORTABLE ${FREERTOS_DIR}/portable/GCC/ARM_CM3/*.c)


file(GLOB USB_CORE Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c)
file(GLOB USB_CLASS_DFU Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Src/*.c)
file(GLOB HAL_SOURCES Drivers/${MCU_FAMILY}_HAL_DRIVER/Src/*.c)
file(GLOB CMSIS_SYSTEM src/system_${MCU_FAMILY}.c)
file(GLOB CMSIS_STARTUP Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103x6.s)
file(GLOB LINKER_FILE ${MCU_LINKER_SCRIPT})

set(SOURCE_FILES  ${CMSIS_RTOS} ${FREERTOS_SOURCES} ${FREERTOS_SOURCES_PORTABLE} ${FREERTOS_SOURCES_MEM}
                  ${USB_CORE} ${USB_CLASS_DFU} ${USER_SOURCES} ${HAL_SOURCES} ${CMSIS_SYSTEM} ${CMSIS_STARTUP} ${LINKER_FILE})

include_directories(Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/${MCU_FAMILY}_HAL_Driver/Inc)
include_directories(inc)
include_directories(${FREERTOS_DIR}/include)
include_directories(${FREERTOS_DIR}/portable/GCC/ARM_CM3)
include_directories(${FREERTOS_DIR}/CMSIS_RTOS)

include_directories(inc/cmsis_rtos)
include_directories(inc/freertos/common)
include_directories(inc/freertos/test)
include_directories(inc/freertos/portable)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Core/Inc)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc)




file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})
set(BUILD_TARGET ${PROJECT_NAME}.elf)

set_target_properties(${BUILD_TARGET} PROPERTIES LINK_FLAGS 
    "-mcpu=${MCU_ARCH}\
    -mthumb\
    -O0\
    -specs=nosys.specs\
    -Wl,-gc-sections -T${PROJECT_SOURCE_DIR}/${MCU_LINKER_SCRIPT}")
target_compile_options(${BUILD_TARGET} PUBLIC 
                             -mcpu=${MCU_ARCH}
                             -mthumb -mthumb-interwork
                              -g -O0 
                             -ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
set(LIST_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.lst)
set(ELF_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.elf)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMAND ${CMAKE_SIZE} --format=berkeley ${PROJECT_NAME}.elf
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
                COMMAND    ${CMAKE_COMMAND}
                ARGS       -E copy $<TARGET_FILE:${PROJECT_NAME}.elf> ${ELF_FILE}
            )
