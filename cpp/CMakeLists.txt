cmake_minimum_required(VERSION 3.18)
project(heuristics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)


add_library(core STATIC
    src/beam_search.cpp
    src/grasp.cpp
    src/local_search.cpp
    src/neighborhoods.cpp
    src/solution.cpp
    src/utils.cpp
    src/construction.cpp
    src/instance.cpp
    src/sa.cpp
    src/vnd.cpp
)

target_include_directories(core PUBLIC src/)

add_executable(main src/main.cpp)
add_executable(tests src/tests.cpp)


target_link_libraries(main PRIVATE core)
target_link_libraries(tests PRIVATE core)
