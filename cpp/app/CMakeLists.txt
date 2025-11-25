add_executable(main main.cpp)
target_link_libraries(main PRIVATE core)

add_executable(tests tests.cpp)
target_link_libraries(tests PRIVATE core)