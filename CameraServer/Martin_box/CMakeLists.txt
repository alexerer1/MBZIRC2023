cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0")


find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(boxDetection STATIC boxDetection.cpp boxDetection.hpp)

add_executable(main main.cpp)

target_link_libraries(main PUBLIC ${OpenCV_LIBS} boxDetection )