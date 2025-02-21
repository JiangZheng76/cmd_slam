cmake_minimum_required(VERSION 3.0)
project(mysylar)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS}  -O0 -fPIC -ggdb -std=c++11 -Wall -Wno-deprecated -Werror -Wno-unused-function -Wno-builtin-macro-redefined -Wno-deprecated-declarations")
# set(CMAKE_CXX_FLAGS "-std=c++11 -s")
include_directories(mysylar)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)


set(STREAMS_SRC
    mysylar/streams/socket_stream.cc 
    mysylar/streams/stream.cc 
    mysylar/streams/zlib_stream.cc
)
set(HTTP_SRC
    mysylar/http/http_connection.cc 
    mysylar/http/http_parser.cc 
    mysylar/http/http_server.cc 
    mysylar/http/http_session.cc 
    mysylar/http/http.cc 
    mysylar/http/http11_parser.cc 
    mysylar/http/httpclient_parser.cc
    mysylar/http/uri.cc
)

set(SYALR_SRC
    mysylar/address.cc 
    mysylar/bytearray.cc 
    mysylar/config.cc 
    mysylar/fd_manager.cc 
    mysylar/fiber.cc 
    mysylar/hook.cc 
    mysylar/iomanager.cc 
    mysylar/log.cc 
    mysylar/marco.cc 
    mysylar/mutex.cc 
    mysylar/scheduler.cc 
    mysylar/servlet.cc 
    mysylar/socket.cc 
    mysylar/tcp_server.cc 
    mysylar/thread.cc 
    mysylar/timer.cc 
    mysylar/util.cc
)

set(LIB_SRC 
    ${SYALR_SRC}
    ${STREAMS_SRC}
    ${HTTP_SRC}
)

# include_directories(${PROJECT_SOURCE_DIR}/lib)
# link_directories(${PROJECT_SOURCE_DIR}/lib)
add_library(mysylar SHARED ${LIB_SRC}) # 构建动态库
#add_library(mysylar STATIC ${LIB_SRC}) # 构建静态库

find_library(PTHREAD pthread)

set(LIB 
    mysylar
    dl
    pthread
    yaml-cpp
    ${OPENSSL_LIBRARIES}
    z
    )

add_executable(config_thread mytest/test_thread.cc)
add_dependencies(config_thread mysylar)
target_link_libraries(config_thread ${LIB})

add_executable(config_test mytest/test_config.cc)
add_dependencies(config_test mysylar)
target_link_libraries(config_test ${LIB})

add_executable(config_util mytest/test_util.cc)
add_dependencies(config_util mysylar)
target_link_libraries(config_util ${LIB})

add_executable(config_fiber mytest/test_fiber.cc)
add_dependencies(config_fiber mysylar)
target_link_libraries(config_fiber ${LIB})

add_executable(config_scheduler mytest/test_scheduler.cc)
add_dependencies(config_scheduler mysylar)
target_link_libraries(config_scheduler ${LIB})

add_executable(config_iomanager mytest/test_iomanager.cc)
add_dependencies(config_iomanager mysylar)
target_link_libraries(config_iomanager ${LIB})

add_executable(config_hook mytest/test_hook.cc)
add_dependencies(config_hook mysylar)
target_link_libraries(config_hook ${LIB})

add_executable(config_address mytest/test_address.cc)
add_dependencies(config_address mysylar)
target_link_libraries(config_address ${LIB})

add_executable(config_socket mytest/test_socket.cc)
add_dependencies(config_socket mysylar)
target_link_libraries(config_socket ${LIB})

add_executable(config_bytearray mytest/test_bytearray.cc)
add_dependencies(config_bytearray mysylar)
target_link_libraries(config_bytearray ${LIB})

add_executable(config_http mytest/test_http.cc)
add_dependencies(config_http mysylar)
target_link_libraries(config_http ${LIB})

add_executable(config_parser mytest/test_parser.cc)
add_dependencies(config_parser mysylar)
target_link_libraries(config_parser ${LIB})

add_executable(config_tcp_server mytest/test_tcp_server.cc)
add_dependencies(config_tcp_server mysylar)
target_link_libraries(config_tcp_server ${LIB})

add_executable(config_echo_tcp_server examples/echo_tcp_server.cc)
add_dependencies(config_echo_tcp_server mysylar)
target_link_libraries(config_echo_tcp_server ${LIB})

add_executable(config_http_server mytest/test_http_server.cc)
add_dependencies(config_http_server mysylar)
target_link_libraries(config_http_server ${LIB})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
