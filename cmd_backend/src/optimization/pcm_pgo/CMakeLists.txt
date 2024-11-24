cmake_minimum_required(VERSION 3.11)
project(pcm_pgo)
set(PCM_INCLUDE_FILE
    ${CMAKE_SOURCE_DIR}/src
)
message(${PCM_INCLUDE_FILE})
set(GTSAM_INCLUDE_DIR /home/sysu/gtsam)
set(GTSAM_LIB /home/sysu/gtsam/build/gtsam/libgtsam.so.4.3a0)
message(${GTSAM_INCLUDE_DIR})

include_directories(
    ${GTSAM_INCLUDE_DIR}
    ${PCM_INCLUDE_FILE}
)

add_library(pcm_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/pcm_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pcm.cpp
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/max_clique_finder)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

set(PCM_LIBS
    ${GTSAM_LIB}
    rpgo_max_clique
)

target_include_directories(pcm_solver PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(pcm_solver PRIVATE ${PCM_LIBS})
