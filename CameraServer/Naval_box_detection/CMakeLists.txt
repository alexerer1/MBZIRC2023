cmake_minimum_required(VERSION 3.0.0)
project(OpenCVTest VERSION 0.1.0)

include(CTest)
include_directories(./include)
include_directories(./src)

FILE(GLOB SOURCES "main.cpp" )
enable_testing()


set(OpenCV_DIR "C:\\opencv\\build")

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(OpenCVTest ${SOURCES} )
#add_executable(OpenCVTest main.cpp)


target_link_libraries( OpenCVTest ${OpenCV_LIBS} )

add_library(boxDetection STATIC ./src/navalBoxDetection.cpp ./include/navalBoxDetection.hpp)

target_link_libraries(${PROJECT_NAME} boxDetection)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
