cmake_minimum_required(VERSION 3.16)
find_package(Eigen3 REQUIRED)

add_executable(test_kf_exe src/test_kf.cpp  src/KFAdapter.cpp 
                                            src/EKFAdapter.cpp
                                            ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/ConstantVelocityModel.cpp 
                                            ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/CoordinatedTurnModel.cpp
                                            ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/CvMeasurementModel.cpp 
                                            ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/DualBearingMeasModel.cpp)

target_include_directories ( test_kf_exe PUBLIC 
                             ${CMAKE_CURRENT_LIST_DIR}/src/   
                             ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/
                             ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/
                            )
target_link_libraries(test_kf_exe PRIVATE Eigen3::Eigen)