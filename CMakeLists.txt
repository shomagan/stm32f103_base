cmake_minimum_required(VERSION 3.6)
set(PRJ_NAME semistor)
include(STM32Toolchain.cmake)

set(MCU_FAMILY STM32F1xx)
set(MCU_LINE STM32F103xx)
set(MCU_TARGET STM32F103c8t6)

project(${PRJ_NAME} C CXX ASM)

add_definitions(-D${MCU_LINE})
add_definitions(-DUSE_HAL_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#1.freertos source and headers files start
#check freertos
set(SOURCES "")
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/freertos)
add_subdirectory(freertos)
set(FREERTOS_SOURCES ${SOURCES})
#headers
file(GLOB FREERTOS_HEADERS  freertos/inc/*.h)
file(GLOB FREERTOS_HEADERS  ${FREERTOS_HEADERS} freertos/inc/portable/*.h)
file(GLOB FREERTOS_HEADERS  ${FREERTOS_HEADERS} freertos/cmsis_os/*.h)
#freertos source and headers files END

file(GLOB_RECURSE USER_SOURCES src/*.c)
file(GLOB HAL_SOURCES Drivers/${MCU_FAMILY}_HAL_DRIVER/Src/*.c)
file(GLOB CMSIS_SYSTEM src/system_${MCU_FAMILY}.c)
file(GLOB CMSIS_STARTUP Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103x6.s)
file(GLOB LINKER_FILE ${MCU_LINKER_SCRIPT})

set(SOURCE_FILES  ${USER_SOURCES} ${HAL_SOURCES} ${CMSIS_SYSTEM} ${CMSIS_STARTUP} ${LINKER_FILE})

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build)
set(BUILD_TARGET ${PROJECT_NAME}.elf)

add_library(freertos_lib ${FREERTOS_SOURCES} ${FREERTOS_HEADERS})
add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES} ${FREERTOS_HEADERS})
target_link_libraries(${PROJECT_NAME}.elf  freertos_lib)

#freertos_lib use only self headers
target_include_directories(freertos_lib PUBLIC freertos/inc)
target_include_directories(freertos_lib PUBLIC freertos/inc/cmsis_os)
target_include_directories(freertos_lib PUBLIC freertos/inc/portable)
target_include_directories(freertos_lib PUBLIC Drivers/CMSIS/Include)
#self 
target_include_directories(${PROJECT_NAME}.elf PUBLIC freertos/inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC freertos/cmsis_os)
target_include_directories(${PROJECT_NAME}.elf PUBLIC freertos/inc/portable)

target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/CMSIS/Include)
target_include_directories(${PROJECT_NAME}.elf PUBLIC Drivers/${MCU_FAMILY}_HAL_Driver/Inc)
target_include_directories(${PROJECT_NAME}.elf PUBLIC inc)

set(MAP_FILE ${PROJECT_SOURCE_DIR}/build/output.map)

set_target_properties(${BUILD_TARGET} PROPERTIES LINK_FLAGS 
    "-mcpu=${MCU_ARCH}\
    -mthumb\
    -O0\
    -specs=nosys.specs\
    -Wl,-gc-sections -T${PROJECT_SOURCE_DIR}/${MCU_LINKER_SCRIPT}\
    -Wl,-Map,${MAP_FILE}")
target_compile_options(${BUILD_TARGET} PUBLIC 
                             -mcpu=${MCU_ARCH}
                             -mthumb -mthumb-interwork
                              -g -O2
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
