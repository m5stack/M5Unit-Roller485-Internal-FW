set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Locate Arm GNU Toolchain from PATH or an optional installation directory.
set(ARM_GNU_TOOLCHAIN_PATH "" CACHE PATH "Arm GNU Toolchain directory (or its bin directory)")
if(NOT ARM_GNU_TOOLCHAIN_PATH AND DEFINED ENV{ARM_GNU_TOOLCHAIN_PATH})
    file(TO_CMAKE_PATH "$ENV{ARM_GNU_TOOLCHAIN_PATH}" ARM_GNU_TOOLCHAIN_PATH)
endif()

set(_ARM_TOOLCHAIN_HINTS)
if(ARM_GNU_TOOLCHAIN_PATH)
    list(APPEND _ARM_TOOLCHAIN_HINTS
        "${ARM_GNU_TOOLCHAIN_PATH}"
        "${ARM_GNU_TOOLCHAIN_PATH}/bin"
    )
endif()

find_program(ARM_GCC NAMES arm-none-eabi-gcc arm-none-eabi-gcc.exe
    HINTS ${_ARM_TOOLCHAIN_HINTS} REQUIRED)
find_program(ARM_GXX NAMES arm-none-eabi-g++ arm-none-eabi-g++.exe
    HINTS ${_ARM_TOOLCHAIN_HINTS} REQUIRED)
find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy arm-none-eabi-objcopy.exe
    HINTS ${_ARM_TOOLCHAIN_HINTS} REQUIRED)
find_program(ARM_SIZE NAMES arm-none-eabi-size arm-none-eabi-size.exe
    HINTS ${_ARM_TOOLCHAIN_HINTS} REQUIRED)

set(CMAKE_C_COMPILER ${ARM_GCC})
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER ${ARM_GXX})
set(CMAKE_LINKER ${ARM_GXX})
set(CMAKE_OBJCOPY ${ARM_OBJCOPY})
set(CMAKE_SIZE ${ARM_SIZE})


set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
