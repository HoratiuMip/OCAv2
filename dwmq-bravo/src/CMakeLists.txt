FILE( GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.* )

idf_component_register( SRCS ${app_sources} )

# ===== regolith-lain BAS begin =====
# Location of regolith-lain.
set( RGH_ROOT_DIR "/home/mip/Lib/regolith-lain" )

# Build target plate.
# - "uC" - target is a microcontroller device.
# - "OS" - target is an operating system ready device. 
set( RGH_TARGET_PLATE "uC" )

# Target operating system.
# - OSp:
#   - "Windows"
# - uCp:
#   - <empty>
#   - "FreeRTOS"
set( RGH_TARGET_OS "FreeRTOS" ) 

# Target platform.
# - OSp:
#   - <empty>
# - uCp:
#   - "esp32"
set( RGH_TARGET_PLATFORM "esp32" )

# * Which external components to make, indicated by a string containing 'y' and 'n' ( for "yes" and "no" ), in the order in which they are built in the script.
# Example: "ynnny" - makes the first and fifth external component.
set( RGH_EXCOM_POPCOUNT
    "n"
)

# Add regolith-lain.
add_subdirectory( "${RGH_ROOT_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/regolith-lain" )
# ===== regolith-lain BAS end =====

target_link_libraries( ${COMPONENT_LIB} PRIVATE RGH_uCp )