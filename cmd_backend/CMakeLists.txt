cmake_minimum_required (VERSION 2.8)
project(cmd_backend)
cmake_policy(SET CMP0048 NEW)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)



# 需要先编译 cmd_comm
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GTest REQUIRED)
set(GTSAM_INCLUDE_DIR /home/sysu/gtsam)
set(GTSAM_LIB /home/sysu/gtsam/build/gtsam/libgtsam.so.4.3a0)
# find_package(Eigen3 REQUIRED)

set(EIGEN3_INCLUDE_DIRS /home/sysu/eigen-3.4.0)

set(PCM_INCLUDE_DIRS ${PCM_PATH}/include())
add_subdirectory(${CMAKE_SOURCE_DIR}/src/optimization/pcm_pgo)

include_directories(
    ${PCM_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/thrid-party/Kimera-RPGO/include
    ${cmd_comm_INCLUDE_DIRS} # 只使用 cmd_comm, sylar 已经包含在了 cmd_comm 中
)
set(LIBS
    fmt
    pthread
    rpgo_max_clique
    yaml-cpp
    pcm_solver
    ${PCM_LIBRARY}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${catkin_LIBRARIES} # mysyalr 的库在 cmd_comm 中已经定义了路径了
    ${Pangolin_LIBRARIES}
    ${GTSAM_LIB}
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
    src/optimization/pcm_pgo/pcm_solver.cpp
    src/optimization/pcm_pgo/pcm.cpp

    src/visualization/agent_display.cpp
    src/visualization/pangolin_viewer.cpp
    src/visualization/loopframe_display.cpp
)

# cs_add_library(cmd_backend ${BACKEND_SOURCE_FILES})
# target_link_libraries(cmd_backend ${LIBS} )

cs_add_executable(cmd_backend_node cmd_sys/cmd_backend_node.cpp ${BACKEND_SOURCE_FILES})
target_link_libraries(cmd_backend_node ${LIBS})
