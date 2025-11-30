add_executable(main main.cpp)
target_link_libraries(main PRIVATE core)

add_executable(tests tests.cpp)
target_link_libraries(tests PRIVATE core)

add_executable(lambda_tuning lambda_tuning.cpp)
target_link_libraries(lambda_tuning PRIVATE core)

add_executable(beam_width beam_width_tuning.cpp)
target_link_libraries(beam_width PRIVATE core)

add_executable(ls ls.cpp)
target_link_libraries(ls PRIVATE core)

add_executable(sa_tuning sa_tuning.cpp)
target_link_libraries(sa_tuning PRIVATE core)

add_executable(ls_timing local_search_timing.cpp)
target_link_libraries(ls_timing PRIVATE core)

add_executable(vnd vnd_order.cpp)
target_link_libraries(vnd PRIVATE core)

add_executable(rho_gamma manipulate_rho_gamma.cpp)
target_link_libraries(rho_gamma PRIVATE core)