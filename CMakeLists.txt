cmake_minimum_required(VERSION 3.16)
set(CMAKE_SYSTEM_NAME Generic)

# turn off compiler checking (compiler is working):
set(CMAKE_C_COMPILER_WORKS TRUE)
# set C standard:
set(CMAKE_C_STANDARD 17)
# set directory for binary files (variable):
set(BUILD_DIR ${CMAKE_SOURCE_DIR}/bin)
# set path for build files:
set(EXECUTABLE_OUTPUT_PATH ${BUILD_DIR})
# create targets variables:
set(TARGET "main")
set(TARGET_ELF "${TARGET}.elf")
set(TARGET_HEX "${TARGET}.hex")
set(TARGET_BIN "${TARGET}.bin")

project(BDSHOT_RPM_EXAMPLE C ASM)

# create main SRC variable:
set(MAIN_SRC_DIR "${CMAKE_SOURCE_DIR}/Src")

set(LINKER_DIR "${CMAKE_SOURCE_DIR}/link")

set(LD_include "-lc -lm -lnosys -L${LINKER_DIR}")

set(linker_script "${LINKER_DIR}/STM32F405RGTx_FLASH.ld")

set(MCU_flags "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

# C defines (defines parse with cmd line not defined in files):
set(C_DEFS "-DSTM32F405xx")

set(C_flags "${MCU_flags} ${C_DEFS} -Wall -fdata-sections -ffunction-sections -DARM_MATH_CM4 -fanalyzer")
set(AS_flags "${MCU_flags} -Wall -fdata-sections -ffunction-sections")
set(LD_flags "${MCU_flags} -specs=nano.specs -specs=nosys.specs  -T${linker_script} ${LD_include} -Wl,--print-memory-usage -u _printf_float ")


add_executable(${TARGET_ELF}
Src/bdshot.c
Src/filters.c
Src/global_variables.c
Src/main.c
Src/setup.c
Drivers/startup_stm32f405xx.s
Drivers/system_stm32f4xx.c
)

target_include_directories(${TARGET_ELF} PUBLIC
Src
Drivers
Drivers/Include
Drivers/CMSIS/DSP/Include
Drivers/STM32F4xx/Include
)

set(CMAKE_C_FLAGS "${C_flags}")
set(CMAKE_ASM_FLAGS "${AS_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${LD_flags}")

target_link_libraries(${TARGET_ELF} PUBLIC "${CMAKE_SOURCE_DIR}/Drivers/libarm_cortexM4lf_math.a")
target_link_libraries(${TARGET_ELF} PUBLIC c m)

# set shourtcut for command:
set(OBJCOPY arm-none-eabi-objcopy)
# make new targets .hex and .bin from .elf file:
add_custom_target(${TARGET_BIN} ALL COMMAND ${OBJCOPY} -O binary -S ${BUILD_DIR}/${TARGET_ELF} ${BUILD_DIR}/${TARGET_BIN})
add_custom_target(${TARGET_HEX} ALL COMMAND ${OBJCOPY} -O ihex -S ${BUILD_DIR}/${TARGET_ELF} ${BUILD_DIR}/${TARGET_HEX})

# define dependeces so that .hex dile is created after .elf and .bin as the last one:
add_dependencies( ${TARGET_HEX} ${TARGET_ELF})
add_dependencies(${TARGET_BIN} ${TARGET_ELF} ${TARGET_HEX})


