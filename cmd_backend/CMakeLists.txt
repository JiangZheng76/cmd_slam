cmake_minimum_required (VERSION 2.8)
project(cmd_backend)
cmake_policy(SET CMP0048 NEW)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(
    -pthread -pipe -Wall -Wextra -pedantic 
    -Wno-unused-parameter -Wno-unused-function -Wno-vla
    -march=native
)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17  ")
IF(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE RelWithDebInfo)
  set(CMAKE_BUILD_TYPE Debug)
ENDIF()

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
# find_package(GTSAM REQUIRED)
# find_package(Eigen3 REQUIRED)


set(EIGEN3_INCLUDE_DIRS /home/sysu/eigen-3.4.0)
message(${EIGEN3_INCLUDE_DIRS})

set(GTSAM_INCLUDE_DIR /home/sysu/gtsam)
set(GTSAM_LIB /home/sysu/gtsam/build/gtsam/libgtsam.so.4.3a0)
message(${GTSAM_INCLUDE_DIR})

set(PCM_PATH /home/sysu/cmd_ws/src/cmd-slam/cmd_backend/thrid-party/Kimera-RPGO)
find_library(PCM_LIBRARY KimeraRPGO ${PCM_PATH}/build)
set(PCM_INCLUDE_DIRS ${PCM_PATH}/include())

include_directories(
    ${PCM_INCLUDE_DIRS}
    src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${cmd_comm_INCLUDE_DIRS} # 只使用 cmd_comm, sylar 已经包含在了 cmd_comm 中
)
set(LIBS
    fmt
    pthread
    ${GTSAM_LIB}
    yaml-cpp
    ${PCM_LIBRARY}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
    ${catkin_LIBRARIES} # mysyalr 的库在 cmd_comm 中已经定义了路径了
    ${Pangolin_LIBRARIES}
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

    src/pcm/pcm_solver.cpp

    src/visualization/agent_display.cpp
    src/visualization/pangolin_viewer.cpp
    src/visualization/loopframe_display.cpp

    # src/KimeraRPGO/Logger.cpp
    # src/KimeraRPGO/RobustSolver.cpp
    # src/KimeraRPGO/GenericSolver.cpp

    # src/KimeraRPGO/utils/GraphUtils.cpp

    # src/KimeraRPGO/max_clique_finder/findClique.cpp
    # src/KimeraRPGO/max_clique_finder/findCliqueHeu.cpp
    # src/KimeraRPGO/max_clique_finder/graphIO.cpp
    # src/KimeraRPGO/max_clique_finder/utils.cpp
)

cs_add_library(cmd_backend ${BACKEND_SOURCE_FILES})
target_link_libraries(cmd_backend ${LIBS} )

cs_add_executable(cmd_backend_node cmd_sys/cmd_backend_node.cpp)
target_link_libraries(cmd_backend_node cmd_backend ${LIBS})
