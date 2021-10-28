cmake_minimum_required(VERSION 3.0.2)
project(camera_calibration)

file(GLOB SOURCES
    camera_calibration.h
    camera_calibration.cpp
)

FIND_PACKAGE( OpenCV REQUIRED )                              
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )


add_executable(camera_calibration camera_calibration.cpp camera_calibration.h)
target_link_libraries(camera_calibration ${OpenCV_LIBS})

