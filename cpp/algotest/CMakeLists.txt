add_executable(main_a main_a.cpp)
target_link_libraries(main_a PRIVATE core)

add_executable(main_b main_b.cpp)
target_link_libraries(main_b PRIVATE core)