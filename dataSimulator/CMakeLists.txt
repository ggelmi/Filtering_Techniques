find_package(Eigen3 REQUIRED)

add_executable(data_sim test_data_sim.cpp Mvn.cpp)

target_link_libraries(data_sim PRIVATE Eigen3::Eigen)
