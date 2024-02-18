cmake_minimum_required(VERSION 2.8)
project( opencv-process-project )

find_library(wiringPi_LIB wiringPi)
find_package( OpenCV REQUIRED )
find_package(CURL REQUIRED) 

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${WIRINGPI_INCLUDE_DIRS}) 
include_directories(${CURL_INCLUDE_DIR})

add_executable( opencv-process opencv-process.cpp )
target_link_libraries( opencv-process ${CURL_LIBRARIES} ${OpenCV_LIBS} ${wiringPi_LIB}) 
