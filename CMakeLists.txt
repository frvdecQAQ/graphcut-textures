cmake_minimum_required(VERSION 3.17)
project(project)

set(CMAKE_CXX_STANDARD 11)
include_directories(./include/)

add_executable(GraphCut-Textures main.cpp include/texture.h src/texture.cpp)

find_package(OpenCV)

target_link_libraries(GraphCut-Textures ${OpenCV_LIBS})
