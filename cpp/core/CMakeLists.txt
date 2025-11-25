add_library(core STATIC)

target_include_directories(
    core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(core
    PRIVATE
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
    src/clustering.cpp
    src/random.cpp
)
