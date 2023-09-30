cmake_minimum_required(VERSION 3.16.3)

# Set the cross-compiler paths or select from idea
#[[set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)]]

#
## Search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
#
## Libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


project(BME680
        VERSION 1.0.0
        DESCRIPTION "BME APP")


## Append the version number to the target name
set(TARGET_NAME "${PROJECT_NAME}_${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")


# Set the target architecture and compiler flags
set(CMAKE_OSX_ARCHITECTURES "armv7-a")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mtune=cortex-a7 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -fdata-sections -fno-builtin -fno-common")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mtune=cortex-a7 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2")

include_directories(src)
include_directories(lib)


# Specify the path to the BME68x library directory
link_directories(lib)

add_executable(BME680
        src/bsec_iot_example/bsec_integration.c
        src/bsec_iot_example/bsec_integration.h
        src/bsec_iot_example/bsec_iot_example.c
        src/bme68x/bme68x.c
        src/bme68x/bme68x.h
        src/bme68x/bme68x_defs.h
        src/config/Selectivity_Config.c
        src/config/Selectivity_Config.h
        src/inc/bsec_datatypes.h
        src/inc/bsec_interface.h)

# algobsec
target_link_libraries(BME680 PRIVATE algobsec) # Adjust library names as needed
# Link against the math library
target_link_libraries(BME680 PRIVATE m)
# Add the linker option to suppress enum size warnings
target_link_options(BME680 PRIVATE -Xlinker -no-enum-size-warning)
# Add -fPIC flag to ensure position-independent code
target_compile_options(BME680 PRIVATE -fPIC)


# Get the version components
string(TOLOWER "${PROJECT_NAME}_${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}" TARGET_NAME)


# Set C++ standard and linker flags
set_target_properties(BME680 PROPERTIES
        OUTPUT_NAME "${TARGET_NAME}"
        CXX_STANDARD 11  # Use the appropriate C++ standard version
        CXX_STANDARD_REQUIRED YES
        LINK_FLAGS "-static"
)
