cmake_minimum_required (VERSION 2.8)
project(cmd_comm)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE release)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-pthread -pipe 
-Wall -Wextra -pedantic 
-Wno-unused-parameter -Wno-unused-function -Wno-vla
-march=native
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17  ")

# 让目标调用库可以找到头文件
set(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/thrid-party/mysylar/mysylar
    ${PROJECT_SOURCE_DIR}/thrid-party/cereal/include
)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()
set(EIGEN3_INCLUDE_DIRS /home/sysu/eigen-3.4)
# find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

find_library(SYLAR mysylar ${PROJECT_SOURCE_DIR}/thrid-party/mysylar/lib/)
set(LIBS
    ${SYLAR}
    pthread
    fmt::fmt
)
include_directories(
    ${INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(cmd_comm_INCLUDE_DIRS ${INCLUDE_DIRS})
set(cmd_comm_LIBRARIES ${LIBS})

set(COMM_SOURCE_FILES
    src/msgs/msg_loopframe.cpp
    src/comm_base.cpp
)
set(COMM_HEADER_FILES
    src/msgs/msg_loopframe.hpp
    src/comm_base.hpp
    src/cmd_comm.hpp
    src/typedefs.hpp

)

cs_add_library(cmd_comm ${COMM_SOURCE_FILES} ${COMM_HEADER_FILES})
target_link_libraries(cmd_comm ${catkin_LIBRARIES} ${LIBS} )

cs_install()
cs_export()