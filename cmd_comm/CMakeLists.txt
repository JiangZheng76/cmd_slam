cmake_minimum_required(VERSION 2.8.3)
project(cmd_comm)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE release)
set(CMAKE_BUILD_TYPE debug)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)



# add_compile_options(-std=c++14 -pthread -pipe -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-function -Wno-vla)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fPIC")

# 让目标调用库可以找到头文件
set(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/thrid-party/mysylar/mysylar
    ${PROJECT_SOURCE_DIR}/thrid-party/cereal/include
)

IF(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

find_package(Eigen3 3.3.4 EXACT REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenSSL REQUIRED)


# 编译 mysylar
add_subdirectory(${PROJECT_SOURCE_DIR}/thrid-party/mysylar)
# ??? 所以每一层都需要加上 需要链接的包才可以么？在最后一层加是不行的
set(SYLAR_LIBS
    fmt
    dl
    z
    pthread
    yaml-cpp
    ${OPENSSL_LIBRARIES}
)
set(LIBS
    # ${SYLAR}
    mysylar
    pthread
    ${SYLAR_LIBS}
)
include_directories(
    ${INCLUDE_DIRS}
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