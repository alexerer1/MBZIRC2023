cmake_minimum_required(VERSION 2.8)
project(camera_server)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0 -pthread")

include_directories(./include)

include_directories(./Martin_box)
include_directories(./Jacob_box)

include_directories(/usr/include/libxml2)

FILE(GLOB SOURCES "src/*.cpp" )

#set( SOURCES src/server_main.cpp)

add_executable(camera_server ${SOURCES} )

#find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package( OpenCV REQUIRED )
#find_package( raspicam REQUIRED )

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(boxDetection STATIC /home/local/MBZIRC2023/CameraServer/Martin_box/boxDetection.cpp /home/local/MBZIRC2023/CameraServer/Martin_box/boxDetection.hpp)
add_library(boxDetection2 STATIC /home/local/MBZIRC2023/CameraServer/Jacob_box/BoxDetection.cpp /home/local/MBZIRC2023/CameraServer/Jacob_box/BoxDetection.h)

#target_link_libraries(${PROJECT_NAME} raspicam raspicam_cv)
#target_link_libraries(${PROJECT_NAME} mmal mmal_core mmal_util)
#target_link_libraries(${PROJECT_NAME} ${raspicam_CV_LIBS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} opencv_imgcodecs)

target_link_libraries(${PROJECT_NAME} boxDetection)
target_link_libraries(${PROJECT_NAME} boxDetection2)
target_link_libraries(${PROJECT_NAME} xml2)

target_link_libraries(${PROJECT_NAME} wiringPi)
target_link_libraries(${PROJECT_NAME} crypt)
target_link_libraries(${PROJECT_NAME} rt)

#target_link_libraries(${PROJECT_NAME} /home/local/MBZIRC2023/CameraServer/Martin_box/build/libboxDetection.a)
