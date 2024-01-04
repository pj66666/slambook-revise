project(test)
cmake_minimum_required(VERSION 3.5)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/sophus)

# 查找gflags包
find_package(gflags REQUIRED)

# 查找glog包
find_package(Glog REQUIRED)

# 查找PCL库  COMPONENTS 后面跟着的是该软件包的一些子组件，例如 common  visualization io
# 在这里，common 和 visualization 是PCL库的两个子组件，分别包含了一些共享的功能和点云的可视化功能
# io 点云读取
find_package(PCL REQUIRED COMPONENTS common visualization io)

# 查找OpenCV库
find_package(OpenCV REQUIRED)


# 添加头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(pcd_to_bird_eye src/pcd_to_bird_eye.cpp)
target_link_libraries(pcd_to_bird_eye 
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    gflags
    )

add_executable(point_cloud_load_and_vis src/point_cloud_load_and_vis.cpp)
target_link_libraries(point_cloud_load_and_vis 
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    gflags
    )

add_executable(scan_to_range_image src/scan_to_range_image.cpp)
target_link_libraries(scan_to_range_image 
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    gflags
    )


add_library(${PROJECT_NAME} 
    src/kdtree.cpp
    src/octo_tree.cpp
    src/bfnn.cpp
    src/point_cloud_utils.cpp
    )
target_link_libraries(${PROJECT_NAME}
    tbb
    )

add_executable(test_nn src/test_nn.cpp)
target_link_libraries(test_nn 
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    gflags 
    ${PROJECT_NAME}
    gtest
    -lpcl_filters
    )   

add_executable(linear_fitting src/linear_fitting.cpp)
target_link_libraries(linear_fitting
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    gflags 
    ${PROJECT_NAME}
    gtest
    -lpcl_filters
    )   