cmake_minimum_required(VERSION 3.0.2)
project(camera_calibration)

file(GLOB SOURCES
    src/camera_calibration.h
    src/camera_calibration.cpp
)

FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )


add_executable(camera_calibration src/camera_calibration.cpp src/camera_calibration.h)
target_link_libraries(camera_calibration ${OpenCV_LIBS})

