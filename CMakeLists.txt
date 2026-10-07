cmake_minimum_required(VERSION 3.16)
project(liquid_laser_pi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_library(USB_LIBRARY usb-1.0 REQUIRED)

set(OPENNI_ROOT "$ENV{HOME}/ros2_astra_camera_openni")
file(GLOB_RECURSE OPENNI_LIB_CANDIDATES
    "${OPENNI_ROOT}/*/arm64/libOpenNI2.so"
    "${OPENNI_ROOT}/**/arm64/libOpenNI2.so")
file(GLOB_RECURSE OPENNI_HEADER_CANDIDATES
    "${OPENNI_ROOT}/*/OpenNI.h"
    "${OPENNI_ROOT}/**/OpenNI.h")

if(NOT OPENNI_LIB_CANDIDATES)
    message(FATAL_ERROR "Could not find ARM64 libOpenNI2.so under ${OPENNI_ROOT}")
endif()
if(NOT OPENNI_HEADER_CANDIDATES)
    message(FATAL_ERROR "Could not find OpenNI.h under ${OPENNI_ROOT}")
endif()

list(GET OPENNI_LIB_CANDIDATES 0 OPENNI_LIBRARY)
list(GET OPENNI_HEADER_CANDIDATES 0 OPENNI_HEADER)
get_filename_component(OPENNI_INCLUDE_DIR "${OPENNI_HEADER}" DIRECTORY)
get_filename_component(OPENNI_LIB_DIR "${OPENNI_LIBRARY}" DIRECTORY)

set(LASERDOCK_ROOT "$ENV{HOME}/laserdocklib")
file(GLOB_RECURSE LASERDOCK_LIB_CANDIDATES
    "${LASERDOCK_ROOT}/build/liblaserdocklib.so"
    "${LASERDOCK_ROOT}/build/liblaserdocklib.a"
    "${LASERDOCK_ROOT}/build/*/liblaserdocklib.so"
    "${LASERDOCK_ROOT}/build/*/liblaserdocklib.a"
    "${LASERDOCK_ROOT}/build/**/liblaserdocklib.so"
    "${LASERDOCK_ROOT}/build/**/liblaserdocklib.a")

if(NOT LASERDOCK_LIB_CANDIDATES)
    message(FATAL_ERROR "Could not find liblaserdocklib under ${LASERDOCK_ROOT}/build")
endif()

list(GET LASERDOCK_LIB_CANDIDATES 0 LASERDOCK_LIBRARY)
get_filename_component(LASERDOCK_LIB_DIR "${LASERDOCK_LIBRARY}" DIRECTORY)
set(LASERDOCK_INCLUDE_DIR "${LASERDOCK_ROOT}/lib/include")

add_executable(liquid_laser
    src/main.cpp
    src/options.cpp
    src/liquid.cpp
    src/contours.cpp
    src/camera.cpp
    src/laser.cpp
)

target_include_directories(liquid_laser PRIVATE
    src
    "${OPENNI_INCLUDE_DIR}"
    "${LASERDOCK_INCLUDE_DIR}"
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_options(liquid_laser PRIVATE
    -O3
    -DNDEBUG
    -mcpu=cortex-a53
    -ffast-math
)

target_link_libraries(liquid_laser PRIVATE
    "${OPENNI_LIBRARY}"
    "${LASERDOCK_LIBRARY}"
    ${OpenCV_LIBS}
    ${USB_LIBRARY}
    Threads::Threads
    atomic
)

set_target_properties(liquid_laser PROPERTIES
    BUILD_RPATH "${OPENNI_LIB_DIR};${LASERDOCK_LIB_DIR}"
)
