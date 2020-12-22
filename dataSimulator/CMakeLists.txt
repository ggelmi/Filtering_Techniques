cmake_minimum_required(VERSION 3.0)
project(data_sim_exe)

find_package(Eigen3 REQUIRED)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../stateEstimation se_build)

file(GLOB_RECURSE DASIM_SRC_FILES 
     "${CMAKE_CURRENT_LIST_DIR}/utilities/*.cpp"
     "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
     )

file(GLOB_RECURSE DASIM_INC_FILES 
     "${CMAKE_CURRENT_LIST_DIR}/utilities/*.cpp"
     "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp"
     )   

add_library(data_sim_lib ${DASIM_SRC_FILES} ${DASIM_INC_FILES})

target_include_directories ( data_sim_lib PUBLIC 
                             ${CMAKE_CURRENT_LIST_DIR}/src/
                             ${CMAKE_CURRENT_LIST_DIR}/utilities/
                             ${CMAKE_CURRENT_LIST_DIR}/../stateEstimation/motion_models/src/
                             ${CMAKE_CURRENT_LIST_DIR}/../stateEstimation/measurement_models/src/
                            )
target_link_libraries(data_sim_lib se_lib)

add_executable(data_sim_exe "${CMAKE_CURRENT_LIST_DIR}/src/test_data_sim.cpp")

target_include_directories ( data_sim_exe PUBLIC 
                             ${CMAKE_CURRENT_LIST_DIR}/src/
                             ${CMAKE_CURRENT_LIST_DIR}/utilities/
                            )

target_link_libraries(data_sim_exe data_sim_lib)
                                   
