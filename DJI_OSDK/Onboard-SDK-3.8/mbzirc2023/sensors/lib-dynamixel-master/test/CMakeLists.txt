cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_test)

## Check C++11 / C++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(../src/dynamixel/include)

add_executable(dynamixel_test
  main.cpp
  ../src/dynamixel/src/dynamixel/Dynamixel.cpp
  ../src/dynamixel/src/dynamixel/SerialPort.cpp)

target_link_libraries(dynamixel_test
  pthread
  boost_system)
