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
set(CMAKE_PROJECT_NAME ROLLER485)

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

# Add application-owned modules
add_subdirectory(Core/User)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    U8g2_lib/mui.c
    U8g2_lib/mui_u8g2.c
    U8g2_lib/u8g2_arc.c
    U8g2_lib/u8g2_bitmap.c
    U8g2_lib/u8g2_box.c
    U8g2_lib/u8g2_buffer.c
    U8g2_lib/u8g2_button.c
    U8g2_lib/u8g2_circle.c
    U8g2_lib/u8g2_cleardisplay.c
    U8g2_lib/u8g2_d_memory.c
    U8g2_lib/u8g2_d_setup.c
    U8g2_lib/u8g2_font.c
    U8g2_lib/u8g2_fonts.c
    U8g2_lib/u8g2_hvline.c
    U8g2_lib/u8g2_input_value.c
    U8g2_lib/u8g2_intersection.c
    U8g2_lib/u8g2_kerning.c
    U8g2_lib/u8g2_line.c
    U8g2_lib/u8g2_ll_hvline.c
    U8g2_lib/u8g2_message.c
    U8g2_lib/u8g2_polygon.c
    U8g2_lib/u8g2_selection_list.c
    U8g2_lib/u8g2_setup.c
    U8g2_lib/u8log.c
    U8g2_lib/u8log_u8g2.c
    U8g2_lib/u8log_u8x8.c
    U8g2_lib/u8x8_8x8.c
    U8g2_lib/u8x8_byte.c
    U8g2_lib/u8x8_cad.c
    U8g2_lib/u8x8_capture.c
    U8g2_lib/u8x8_d_ssd1306_64x48.c
    U8g2_lib/u8x8_debounce.c
    U8g2_lib/u8x8_display.c
    U8g2_lib/u8x8_fonts.c
    U8g2_lib/u8x8_gpio.c
    U8g2_lib/u8x8_input_value.c
    U8g2_lib/u8x8_message.c
    U8g2_lib/u8x8_selection_list.c
    U8g2_lib/u8x8_setup.c
    U8g2_lib/u8x8_string.c
    U8g2_lib/u8x8_u16toa.c
    U8g2_lib/u8x8_u8toa.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/CMSIS/DSP/Include
    U8g2_lib
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Debug runs as a standalone application at the start of Flash. Release keeps
# the production layout expected by the bootloader.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(APP_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32G431XX_FLASH_DEBUG.ld")
else()
    set(APP_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32G431XX_FLASH.ld")
endif()

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-T${APP_LINKER_SCRIPT}"
    -u _printf_float
)
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS "${APP_LINKER_SCRIPT}"
)
message("Linker script: ${APP_LINKER_SCRIPT}")

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate binary and Intel HEX images next to the ELF after every successful
# link. These files use the same Debug/Release linker layout as the ELF.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
        "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>"
        "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex"
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
    VERBATIM
)

# Expose a Debug-only flash target to the CMake Tools target selector.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(JLINK_COMMANDER "" CACHE FILEPATH "Path to the J-Link Commander executable")
    if(NOT JLINK_COMMANDER)
        set(_JLINK_HINTS)
        foreach(_JLINK_ENV JLINK_PATH JLINK_HOME)
            if(DEFINED ENV{${_JLINK_ENV}})
                list(APPEND _JLINK_HINTS "$ENV{${_JLINK_ENV}}")
            endif()
        endforeach()
        find_program(_JLINK_COMMANDER NAMES JLink JLink.exe
            HINTS ${_JLINK_HINTS}
            NO_CACHE
        )
        if(NOT _JLINK_COMMANDER AND CMAKE_HOST_WIN32 AND DEFINED ENV{ProgramFiles})
            file(GLOB _JLINK_EXECUTABLES
                "$ENV{ProgramFiles}/SEGGER/JLink*/JLink.exe"
            )
            list(SORT _JLINK_EXECUTABLES COMPARE NATURAL ORDER DESCENDING)
            if(_JLINK_EXECUTABLES)
                list(GET _JLINK_EXECUTABLES 0 _JLINK_COMMANDER)
            endif()
        endif()
        if(_JLINK_COMMANDER)
            set(JLINK_COMMANDER "${_JLINK_COMMANDER}" CACHE FILEPATH
                "Path to the J-Link Commander executable" FORCE)
        endif()
    endif()

    if(JLINK_COMMANDER AND EXISTS "${JLINK_COMMANDER}")
        add_custom_target(flash-debug
            COMMAND "${JLINK_COMMANDER}"
                -device STM32G431CB
                -if SWD
                -speed 4000
                -autoconnect 1
                -ExitOnError 1
                -CommanderScript "${CMAKE_SOURCE_DIR}/.vscode/jlink/flash-debug.jlink"
            DEPENDS ${CMAKE_PROJECT_NAME}
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            USES_TERMINAL
            VERBATIM
        )
    else()
        message(STATUS "J-Link Commander not found; flash-debug target is disabled")
    endif()
endif()
