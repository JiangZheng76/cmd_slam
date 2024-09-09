cmake_minimum_required (VERSION 2.8)
project(cmd_frontend)

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -00 -g")
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS /home/sysu/eigen-3.3.7)
find_package(fmt REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)
set(LIBS
    fmt::fmt
    ${catkin_LIBRARIES}
)

set(cmd_frontend_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
# set(cmd_frontend_LIBRARIES ${LIBS})

cs_add_library(cmd_frontend src/communicator.cpp)
target_link_libraries(cmd_frontend ${LIBS})

cs_export()