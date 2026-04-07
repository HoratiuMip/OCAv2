FILE( GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.* )

idf_component_register( SRCS ${app_sources} )

# ===== auto-a113 BAS begin =====
# Location of auto-a113.
set( A113_ROOT_DIR "/home/mip/Lib/auto-a113" )

# Build target plate.
# - "uC" - target is a microcontroller device.
# - "OS" - target is an operating system ready device. 
set( A113_TARGET_PLATE "uC" )

# Target operating system.
# - OSp:
#   - "Windows"
# - uCp:
#   - <empty>
#   - "FreeRTOS"
set( A113_TARGET_OS "FreeRTOS" ) 

# Target platform.
# - OSp:
#   - <empty>
# - uCp:
#   - "esp32"
set( A113_TARGET_PLATFORM "esp32" )

# Clockworks to configure and build.
#  - "immersive" - The Immersive framework, used for GUI and 3D rendering.
set( A113_MAKE_CLOCKWORKS 
    "" 
)

# * Which external components to make, indicated by a string containing 'y' and 'n' ( for "yes" and "no" ), in the order in which they are built in the script.
# Example: "ynnny" - makes the first and fifth external component.
set( A113_EXCOM_POPCOUNT
    "n"
)

# Add auto-a113.
add_subdirectory( "${A113_ROOT_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/auto-a113" )
# ===== auto-a113 BAS end =====

target_link_libraries( ${COMPONENT_LIB} PRIVATE A113_uCp )