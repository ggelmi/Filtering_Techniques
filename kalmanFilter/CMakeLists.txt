find_package(Eigen3 REQUIRED)

add_executable(filter_exe kalmanFilter.cpp)
target_link_libraries(filter_exe PRIVATE Eigen3::Eigen)
