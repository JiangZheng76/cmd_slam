cmake_minimum_required(VERSION 3.0)
project(mysylar)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS}  -O0 -fPIC -ggdb -std=c++14 -Wall -Wno-deprecated -Werror -Wno-unused-function -Wno-builtin-macro-redefined -Wno-deprecated-declarations")
include_directories(mysylar)
find_package(yaml-cpp REQUIRED)

set(LIB_SRC 
    mysylar/config.cc
    mysylar/mutex.cc
    mysylar/util.cc
    mysylar/thread.cc
    mysylar/log.cc
    mysylar/fiber.cc
    mysylar/scheduler.cc
    mysylar/iomanager.cc
    mysylar/timer.cc
    # mysylar/hook.cc
    mysylar/fd_manager.cc
    mysylar/address.cc
    mysylar/socket.cc
    mysylar/bytearray.cc
    mysylar/http.cc
    mysylar/http/http11_parser.cc
    mysylar/http/httpclient_parser.cc
    mysylar/http/http_parser.cc
    mysylar/tcp_server.cc
    mysylar/stream.cc
    mysylar/socket_stream.cc
    mysylar/http_session.cc
    mysylar/http_server.cc
    mysylar/servlet.cc
    )

# include_directories(${PROJECT_SOURCE_DIR}/lib)
# link_directories(${PROJECT_SOURCE_DIR}/lib)
set(SYLAR_LIB ${PROJECT_SOURCE_DIR}/lib)
set(SYLAR_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/mysylar)
add_library(mysylar ${LIB_SRC})

find_library(PTHREAD pthread)

set(LIB 
    mysylar
    dl
    pthread
    yaml-cpp
    )
set(SYLAR_LIB 
    dl
    pthread
    yaml-cpp
    )

target_link_libraries(mysylar ${SYLAR_LIB})

# add_executable(config_thread mytest/test_thread.cc)
# add_dependencies(config_thread mysylar)
# target_link_libraries(config_thread ${LIB})

# add_executable(config_test mytest/test_config.cc)
# add_dependencies(config_test mysylar)
# target_link_libraries(config_test ${LIB})

# add_executable(config_util mytest/test_util.cc)
# add_dependencies(config_util mysylar)
# target_link_libraries(config_util ${LIB})

# add_executable(config_fiber mytest/test_fiber.cc)
# add_dependencies(config_fiber mysylar)
# target_link_libraries(config_fiber ${LIB})

# add_executable(config_scheduler mytest/test_scheduler.cc)
# add_dependencies(config_scheduler mysylar)
# target_link_libraries(config_scheduler ${LIB})

# add_executable(config_iomanager mytest/test_iomanager.cc)
# add_dependencies(config_iomanager mysylar)
# target_link_libraries(config_iomanager ${LIB})

# add_executable(config_hook mytest/test_hook.cc)
# add_dependencies(config_hook mysylar)
# target_link_libraries(config_hook ${LIB})

# add_executable(config_address mytest/test_address.cc)
# add_dependencies(config_address mysylar)
# target_link_libraries(config_address ${LIB})

# add_executable(config_socket mytest/test_socket.cc)
# add_dependencies(config_socket mysylar)
# target_link_libraries(config_socket ${LIB})

# add_executable(config_bytearray mytest/test_bytearray.cc)
# add_dependencies(config_bytearray mysylar)
# target_link_libraries(config_bytearray ${LIB})

# add_executable(config_http mytest/test_http.cc)
# add_dependencies(config_http mysylar)
# target_link_libraries(config_http ${LIB})

# add_executable(config_parser mytest/test_parser.cc)
# add_dependencies(config_parser mysylar)
# target_link_libraries(config_parser ${LIB})

# add_executable(config_tcp_server mytest/test_tcp_server.cc)
# add_dependencies(config_tcp_server mysylar)
# target_link_libraries(config_tcp_server ${LIB})

# add_executable(config_echo_tcp_server examples/echo_tcp_server.cc)
# add_dependencies(config_echo_tcp_server mysylar)
# target_link_libraries(config_echo_tcp_server ${LIB})

# add_executable(config_http_server mytest/test_http_server.cc)
# add_dependencies(config_http_server mysylar)
# target_link_libraries(config_http_server ${LIB})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
