cmake_minimum_required(VERSION 2.8)
project(point_to_point_icp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/local/include/eigen3")

# sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS} "/Users/mark/Desktop/slam-practice/slambook2/3rdparty/Sophus")

# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

link_directories(/usr/local/lib)

add_library(g2o_ICP STATIC g2o_ICP.cpp)
add_executable(test_main test_main.cpp)
target_link_libraries(test_main
        g2o_core g2o_stuff g2o_types_icp g2o_types_slam3d g2o_ICP ${CHOLMOD_LIBRARIES}
        )

#add_executable(read_points read_points.cpp)

#add_executable(test_read_points test_read_points.cpp)

# add_executable(icp_main icp_main.cpp)
# target_link_libraries(icp_main
#         g2o_core g2o_stuff g2o_types_icp g2o_types_slam3d ${CHOLMOD_LIBRARIES}
#         )




