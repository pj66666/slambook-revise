project(test)
cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# include_directories(${PROJECT_SOURCE_DIR}/sophus)

# 查找gflags包
find_package(gflags REQUIRED)

# 查找glog包
find_package(Glog REQUIRED)

find_package(Sophus REQUIRED)

# 添加头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

add_library(other
src/io_utils.cc
src/utm_convert.cc
src/utm.cc
src/tranmerc.cc
src/static_imu_init.cc)




add_executable(run_imu_integration
        exe/run_imu_integration.cc
        )

target_link_libraries(run_imu_integration
        glog 
        gflags
        Sophus::Sophus
        other
        )

add_executable(run_eskf_gins exe/run_eskf_gins.cc)
target_link_libraries(run_eskf_gins
        glog gflags Sophus::Sophus
        other
        )

add_executable(process_gnss exe/process_gnss.cc)
target_link_libraries(process_gnss
        glog gflags Sophus::Sophus
        other
        )

