cmake_minimum_required(VERSION 3.5)
project(nlink_parser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(extern/serial)

set(msg_files
  "msg/LinktrackAnchorframe0.msg"
  "msg/LinktrackNode0.msg"
  "msg/LinktrackNode1.msg"
  "msg/LinktrackNode2.msg"
  "msg/LinktrackNodeframe0.msg"
  "msg/LinktrackNodeframe1.msg"
  "msg/LinktrackNodeframe2.msg"
  "msg/LinktrackNodeframe3.msg"
  "msg/LinktrackNodeframe4.msg"
  "msg/LinktrackNodeframe5.msg"
  "msg/LinktrackNodeframe6.msg"
  "msg/LinktrackTag.msg"
  "msg/LinktrackTagframe0.msg"
  "msg/TofsenseCascade.msg"
  "msg/TofsenseFrame0.msg"
  "msg/LinktrackAoaNode0.msg"
  "msg/LinktrackAoaNodeframe0.msg"
  "msg/LinktrackNode4Anchor.msg"
  "msg/LinktrackNode4Tag.msg"
  "msg/LinktrackNodeframe4.msg"
  "msg/LinktrackNode5.msg"
  "msg/LinktrackNodeframe5.msg"
  "msg/LinktrackNode6.msg"
  "msg/LinktrackNodeframe6.msg"
  "msg/TofsenseMFrame0.msg"
  "msg/TofsenseMFrame0Pixel.msg"
  "msg/IotFrame0.msg"
  "msg/IotFrame0Node.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rosidl_default_runtime
  std_msgs
)

set(utils_sources
  src/utils/protocol_extracter/nprotocol_base.cpp
  src/utils/protocol_extracter/nprotocol_extracter.cpp
  src/utils/nlink_unpack/nlink_utils.c
  src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
  src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
  src/utils/nlink_unpack/nlink_tofsense_frame0.c
  src/utils/nlink_unpack/nlink_tofsensem_frame0.c
  src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
  src/utils/nlink_unpack/nlink_iot_frame0.c
  src/utils/init_serial.cpp
  src/utils/nlink_protocol.cpp
  src/utils/nutils.cpp
)

add_library(nutils ${utils_sources})

target_include_directories(nutils PUBLIC
  src/utils
  extern/serial/include
)

ament_target_dependencies(nutils PUBLIC rclcpp std_msgs)

target_link_libraries(nutils PUBLIC serial)

add_executable(linktrack
  src/linktrack/init.cpp
  src/linktrack/main.cpp
  src/linktrack/protocols.cpp
)
target_link_libraries(linktrack nutils)
ament_target_dependencies(linktrack rclcpp std_msgs)
rosidl_target_interfaces(linktrack ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(linktrack_rviz_converter
  src/linktrack/main_rviz_converter.cpp
)
target_link_libraries(linktrack_rviz_converter nutils)
ament_target_dependencies(linktrack_rviz_converter rclcpp geometry_msgs)
rosidl_target_interfaces(linktrack_rviz_converter ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(tofsense
  src/tofsense/init.cpp
  src/tofsense/main.cpp
)
target_link_libraries(tofsense nutils)
ament_target_dependencies(tofsense rclcpp)
rosidl_target_interfaces(tofsense ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(tofsensem
  src/tofsensem/init.cpp
  src/tofsensem/main.cpp
)
target_link_libraries(tofsensem nutils)
ament_target_dependencies(tofsensem rclcpp)
rosidl_target_interfaces(tofsensem ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(linktrack_aoa
  src/linktrack_aoa/init.cpp
  src/linktrack_aoa/main.cpp
  src/linktrack/protocols.cpp
)
target_link_libraries(linktrack_aoa nutils)
ament_target_dependencies(linktrack_aoa rclcpp std_msgs)
rosidl_target_interfaces(linktrack_aoa ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(iot
  src/iot/init.cpp
  src/iot/main.cpp
)
target_link_libraries(iot nutils)
ament_target_dependencies(iot rclcpp)
rosidl_target_interfaces(iot ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  ament_add_gtest(nlink_parser_test
    src/linktrack/init.cpp
    src/linktrack/protocols.cpp
    src/tofsense/init.cpp
    src/linktrack_aoa/init.cpp
    src/tofsensem/init.cpp
    src/iot/init.cpp
    test/test_nlink_parser.cpp
  )
  if(TARGET nlink_parser_test)
    target_link_libraries(nlink_parser_test nutils)
    ament_target_dependencies(nlink_parser_test rclcpp std_msgs)
    rosidl_target_interfaces(nlink_parser_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
endif()

install(TARGETS
  linktrack
  linktrack_rviz_converter
  tofsense
  tofsensem
  linktrack_aoa
  iot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
