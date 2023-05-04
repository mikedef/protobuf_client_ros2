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
find_package(Boost REQUIRED COMPONENTS)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(protobuf_client_interfaces REQUIRED)  # Custom Msg for protobuf_client

message("libprotobuf >= 2.5.0")
set(PROTOBUF_ALLOW_ALIAS "option allow_alias = true;")
set(Protobuf_INCLUDE_DIRS include/protobuf_client)

## Build                                                                                                
include_directories(include include/protobuf_client include/lib_gateway_tcp
			    rclcpp ${Boost_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
			    ${B64_LIBRARIES})

# lib/proto is a directory to store the raw gateway.proto msg 
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS include/protobuf_client/ lib/proto/gateway.proto)

# Library: gateway_protobuf
add_library(gateway_protobuf include/protobuf_client/gateway.pb.cc)

install(
  TARGETS gateway_protobuf
  EXPORT gateway_protobuf
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/gateway_protobuf
  )	   


# Library: gateway_tcp_lib
add_library(gateway_tcp_lib                                                                             
  lib/lib_gateway_tcp/tcp_client.cpp
  lib/lib_gateway_tcp/tcp_server.cpp
  lib/lib_gateway_tcp/tcp_session.cpp
  #include/lib_gateway_tcp/tcp_client.h
  #include/lib_gateway_tcp/tcp_server.h
  #include/lib_gateway_tcp/tcp_session.h
  )

target_link_libraries(gateway_tcp_lib
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  b64)

ament_target_dependencies(gateway_tcp_lib
  ${B64_LIBRARIES}
  ${Boost_LIBRARIES})
ament_export_targets(gateway_tcp_lib HAS_LIBRARY_TARGET)
ament_target_dependencies(gateway_tcp_lib
  PUBLIC)

install(
  TARGETS gateway_tcp_lib 
  EXPORT gateway_tcp_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/lib_gateway_tcp
  )


# Node: protobuf_client_node
add_executable(protobuf_client_node 
  src/protobuf_client_node.cpp)
target_link_libraries(protobuf_client_node
  gateway_tcp_lib
  gateway_protobuf
  ${PROTOBUF_LIBRARY}
  ${Boost_LIBRARIES})
ament_target_dependencies(protobuf_client_node
  rclcpp
  Boost
  class_loader
  std_msgs
  protobuf_client_interfaces)

install(TARGETS
  protobuf_client_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/gateway_tcp_lib)
install(DIRECTORY include/ DESTINATION include/gateway_protobuf)
ament_export_targets(gateway_protobuf HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)


ament_package()
