cmake_minimum_required(VERSION 2.8)

project(PegDetect)

find_file(ZMQLIB libzmq.so /usr/local/lib)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

if(NOT ZMQLIB)
  MESSAGE("GRR NO ZMQ")
endif(NOT ZMQLIB)

add_executable(PegDetect PegDetect.cpp)

target_link_libraries(PegDetect ${OpenCV_LIBS} ${ZMQLIB})
