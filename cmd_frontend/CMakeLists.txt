cmake_minimum_required(VERSION 2.8.3)
project(cmd_frontend)

set(CMAKE_BUILD_TYPE debug)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(cmd_frontend_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)
# set(cmd_frontend_LIBRARIES ${LIBS})

cs_add_library(cmd_frontend STATIC src/communicator.cpp)
target_link_libraries(cmd_frontend ${catkin_LIBRARIES})

cs_export()