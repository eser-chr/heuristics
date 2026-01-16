find_package(OpenMP REQUIRED)


add_executable(main_a main_a.cpp)
target_link_libraries(main_a PRIVATE core)

add_executable(test_fairness test_fairness.cpp)
target_link_libraries(test_fairness PRIVATE core OpenMP::OpenMP_CXX)

add_executable(tuning_ga tuning_ga.cpp)
target_link_libraries(tuning_ga PRIVATE core OpenMP::OpenMP_CXX)

add_executable(tuning_ln tuning_ln.cpp)
target_link_libraries(tuning_ln PRIVATE core OpenMP::OpenMP_CXX)

add_executable(test_algorithm test_algorithm.cpp)
target_link_libraries(test_algorithm PRIVATE core OpenMP::OpenMP_CXX)

add_executable(competition competition.cpp)
target_link_libraries(competition PRIVATE core)

add_executable(test_fairness_impact_ga test_fairness_impact_ga.cpp)
target_link_libraries(test_fairness_impact_ga PRIVATE core OpenMP::OpenMP_CXX)