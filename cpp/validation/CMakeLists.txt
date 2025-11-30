add_executable(lambda_tuning_val lambda_tuning.cpp)
target_link_libraries(lambda_tuning_val PRIVATE core)

add_executable(beam_width_val beam_width_tuning.cpp)
target_link_libraries(beam_width_val PRIVATE core)

add_executable(ls_val ls.cpp)
target_link_libraries(ls_val PRIVATE core)

add_executable(sa_tuning_val sa_tuning.cpp)
target_link_libraries(sa_tuning_val PRIVATE core)

add_executable(vnd_val vnd_order.cpp)
target_link_libraries(vnd_val PRIVATE core)
