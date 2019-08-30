set(project_name_imusimwithpointline)

#project(${project_name})
#cmake_minimum_required(version 2.8)

PROJECT(IMU)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(cmake_build_type Debug)

#set(executable_output_path ${project_source_dir}/bin)
#set(library_output_path ${project_source_dir}/lib)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(cmake_cxx_flags "${cmake_cxx_flags} -std=c++11")
#set(cmake_module_path ${cmake_module_path} "${project_source_dir}/cmake_modules/")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)



#include_directories(${EIGEN3_INCLUDE_DIR} ${SOPHUS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR})
include_directories(
${EIGEN3_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR} # for both sophus and geographiclib
${OPENCV_INCLUDE_DIR}
)

#list(APPEND LINK_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES})
LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
${Sophus_LIBRARIES}
)

ADD_EXECUTABLE(data_gen main/gener_alldata.cpp src/param.h src/param.cpp src/imu.h 
src/imu.cpp src/utilities.h src/utilities.cpp)
#target_link_libraries(data_gen ${LINK_LIBS})
TARGET_LINK_LIBRARIES (data_gen ${LINK_LIBS})