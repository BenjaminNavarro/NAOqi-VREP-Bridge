cmake_minimum_required(VERSION 2.8)
project(naoqi_vrep_bridge)

find_package(qibuild)

find_package(ALPROXIES)
find_package(ALVISION)

include_directories(${ALPROXIES_INCLUDE_DIRS})
include_directories(".")

set(VREP_CFLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -D__linux")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${VREP_CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${VREP_CFLAGS} -std=gnu++11")

qi_create_lib(vrep extApi.c extApiPlatform.c)
qi_stage_lib(vrep)

# Create a executable named naoqi_vrep_bridge
# with the source file: main.cpp
qi_create_bin(naoqi_vrep_bridge "main.cpp")
qi_use_lib(naoqi_vrep_bridge ALPROXIES ${ALPROXIES_DEPENDS} ALVISION ${ALVISION_DEPENDS})
qi_use_lib(naoqi_vrep_bridge vrep)

# Create a executable named naoqi_vrep_bridge_two_cam
# with the source file: main_two_cam.cpp
qi_create_bin(naoqi_vrep_bridge_two_cam "main_two_cam.cpp")
qi_use_lib(naoqi_vrep_bridge_two_cam ALPROXIES ${ALPROXIES_DEPENDS} ALVISION ${ALVISION_DEPENDS})
qi_use_lib(naoqi_vrep_bridge_two_cam vrep)
