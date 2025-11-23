cmake_minimum_required(VERSION 3.18)
project(heuristics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

file(GLOB SOURCES src/*.cpp)

add_executable(tests ${SOURCES})

