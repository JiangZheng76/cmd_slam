cmake_minimum_required(VERSION 2.8.3)
project(cmd_backend)
cmake_policy(SET CMP0048 NEW)

add_compile_options(
    -std=c++1z -pthread -pipe -Wall -Wextra -pedantic 
    -Wno-unused-parameter -Wno-unused-function -Wno-vla
)

IF(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE RelWithDebInfo)
  set(CMAKE_BUILD_TYPE Debug)
ENDIF()

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# 需要先编译 cmd_comm
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3.4 EXACT REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${cmd_comm_INCLUDE_DIRS} # 只使用 cmd_comm, sylar 已经包含在了 cmd_comm 中
)
set(LIBS
    fmt
    dl
    z
    pthread
    yaml-cpp
    ${OPENSSL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${catkin_LIBRARIES} # mysyalr 的库在 cmd_comm 中已经定义了路径了
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

set(BACKEND_SOURCE_FILES
    src/backend.cpp
    src/loopframe.cpp
    src/map.cpp
    src/agent.cpp
    src/communicator.cpp
    src/loop_handler.cpp
    src/loop_closure/scancontext.cpp

    src/optimization/cmd_sim3.cpp
    src/optimization/optimization.cpp

    src/visualization/codsv_visual/AgentDisplay.cpp
    src/visualization/codsv_visual/LoopFrameDisplay.cpp
    src/visualization/codsv_visual/PangolinLoopViewer.cpp
)

add_library(cmd_backend ${BACKEND_SOURCE_FILES})
target_link_libraries(cmd_backend ${LIBS} )

add_executable(cmd_backend_node cmd_sys/cmd_backend_node.cpp)
target_link_libraries(cmd_backend_node cmd_backend ${LIBS})
