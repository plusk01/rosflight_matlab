cmake_minimum_required(VERSION 3.0)
project(rosflight_matlab)

set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Define message colors
string(ASCII 27 ESC)
set(RED "${ESC}[1;31m")
set(BLUE "${ESC}[1;34m")
set(YELLOW "${ESC}[1;33m")
set(RESET "${ESC}[m")

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

##################################
#  Download ROSflight firmware   #
##################################

set(ROSFLIGHT_DIR ${CMAKE_CURRENT_BINARY_DIR}/rosflight-download)

# Download ROSflight firmware
configure_file(
  "${CMAKE_MODULE_PATH}/rosflight.cmake.in"
  "${ROSFLIGHT_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${ROSFLIGHT_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${ROSFLIGHT_DIR})

##################################
#  Configure ROSflight firmware  #
##################################

set(FIRMWARE_DIR ${ROSFLIGHT_DIR}/src)

# git version hash
execute_process(
  COMMAND git rev-parse --short=8 HEAD
  WORKING_DIRECTORY ${FIRMWARE_DIR}
  OUTPUT_VARIABLE GIT_VERSION_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# git version string
execute_process(
  COMMAND git describe --tags --abbrev=8 --always --dirty --long
  WORKING_DIRECTORY ${FIRMWARE_DIR}
  OUTPUT_VARIABLE GIT_VERSION_STRING
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# this list is passed to MEX for building
set(ROSFLIGHT_GIT GIT_VERSION_HASH=0x${GIT_VERSION_HASH} GIT_VERSION_STRING="${GIT_VERSION_STRING}")

# add_library(rosflight_firmware
set(ROSFLIGHT_SOURCES
  ${FIRMWARE_DIR}/src/rosflight.cpp
  ${FIRMWARE_DIR}/src/nanoprintf.cpp
  ${FIRMWARE_DIR}/src/estimator.cpp
  ${FIRMWARE_DIR}/src/mixer.cpp
  ${FIRMWARE_DIR}/src/controller.cpp
  ${FIRMWARE_DIR}/src/param.cpp
  ${FIRMWARE_DIR}/src/state_manager.cpp
  ${FIRMWARE_DIR}/src/rc.cpp
  ${FIRMWARE_DIR}/src/command_manager.cpp
  ${FIRMWARE_DIR}/src/sensors.cpp
  ${FIRMWARE_DIR}/src/comm_manager.cpp
  ${FIRMWARE_DIR}/comms/mavlink/mavlink.cpp
  ${FIRMWARE_DIR}/lib/turbomath/turbomath.cpp
)

# target_include_directories(rosflight_firmware
#     PUBLIC
#         $<INSTALL_INTERFACE:include>
#         $<BUILD_INTERFACE:${FIRMWARE_DIR}/include>
#         $<BUILD_INTERFACE:${FIRMWARE_DIR}/lib>
#         $<BUILD_INTERFACE:${FIRMWARE_DIR}/comms>
# )

##################################
#    Configure MATLAB Board      #
##################################

add_library(rosflight_matlab
    ${ROSFLIGHT_SOURCES}
    src/matlab_board.cpp
)

target_include_directories(rosflight_matlab
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${FIRMWARE_DIR}/include>
        $<BUILD_INTERFACE:${FIRMWARE_DIR}/lib>
        $<BUILD_INTERFACE:${FIRMWARE_DIR}/comms>
)

set_target_properties(rosflight_matlab PROPERTIES
                          POSITION_INDEPENDENT_CODE ON
)
target_compile_definitions(rosflight_matlab PUBLIC ${ROSFLIGHT_GIT})

##################################
#         MATLAB Bindings        #
##################################

set(STATIC_LIB rosflight_matlab)
add_subdirectory(bindings/matlab)
