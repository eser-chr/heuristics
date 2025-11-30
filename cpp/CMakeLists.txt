cmake_minimum_required(VERSION 3.18)
project(heuristics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(core)
add_subdirectory(app)
add_subdirectory(validation)
add_subdirectory(competition)
add_subdirectory(algotest)

