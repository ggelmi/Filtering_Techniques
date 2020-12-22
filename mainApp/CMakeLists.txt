cmake_minimum_required(VERSION 3.0)
project(almis_app_exe)

find_package(Eigen3 REQUIRED)

#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../stateEstimation se_build)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../dataSimulator data_sim_build)

add_executable(almis_app_exe "${CMAKE_CURRENT_LIST_DIR}/src/main.cpp")

target_link_libraries(almis_app_exe data_sim_lib)
                                   
