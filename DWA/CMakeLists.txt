cmake_minimum_required(VERSION 3.5)
project(dwa_cpu_demo)

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 3.2.0 REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
)

add_executable(dwa_cpu_demo dwa_demo.cpp dwa.cpp)

target_link_libraries(dwa_cpu_demo ${OpenCV_LIBS})