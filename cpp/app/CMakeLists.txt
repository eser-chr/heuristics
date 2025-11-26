add_executable(main main.cpp)
target_link_libraries(main PRIVATE core)

add_executable(tests tests.cpp)
target_link_libraries(tests PRIVATE core)

add_executable(lambda_tuning lambda_tuning.cpp)
target_link_libraries(lambda_tuning PRIVATE core)

add_executable(beam_width beam_width_tuning.cpp)
target_link_libraries(beam_width PRIVATE core)

add_executable(stepping_method stepping_method_tuning.cpp)
target_link_libraries(stepping_method PRIVATE core)

add_executable(sa_tuning sa_tuning.cpp)
target_link_libraries(sa_tuning PRIVATE core)