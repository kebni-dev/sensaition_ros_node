find_package(GTest REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_ros_node
  test_kebni_node.cpp
)

target_link_libraries(test_ros_node
    GTest::gtest
    GTest::gtest_main
    Kebni::sensaition_parser
)

target_include_directories(test_ros_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_ros_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

if (WIN32)
  target_link_libraries(test_ros_node PRIVATE ws2_32)
endif()

add_test(NAME kebni_ros_node_tests COMMAND test_ros_node)