cmake_minimum_required(VERSION 3.5)
project(protobuf_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)

message("libprotobuf >= 2.5.0")
set(PROTOBUF_ALLOW_ALIAS "option allow_alias = true;")
set(Protobuf_INCLUDE_DIRS include/protobuf_client)

## Build                                                                                                
include_directories(include include/protobuf_client include/lib_gateway_tcp
			    rclcpp ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})      
# lib/proto is a directory to store the raw gateway.proto msg 
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS include/protobuf_client/ lib/proto/gateway.proto)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gateway.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  )

add_library(gateway_protobuf include/protobuf_client/gateway.pb.cc)

# Library: gateway_tcp_lib
add_library(gateway_tcp_lib                                                                             
  lib/lib_gateway_tcp/tcp_client.cpp
  lib/lib_gateway_tcp/tcp_server.cpp
  lib/lib_gateway_tcp/tcp_session.cpp
  include/lib_gateway_tcp/tcp_client.h
  include/lib_gateway_tcp/tcp_server.h
  include/lib_gateway_tcp/tcp_session.h
  )

ament_export_targets(gateway_tcp_lib HAS_LIBRARY_TARGET)

install(
  TARGETS gateway_tcp_lib 
  EXPORT gateway_tcp_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )


ament_package()
