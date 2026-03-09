cmake_minimum_required(VERSION 3.8)
project(kebni_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_subdirectory(external/sensaition_parser)

add_executable(kebni_driver_node
  src/kebni_node.cpp
)

target_link_libraries(kebni_driver_node
  Kebni::sensaition_parser
)

ament_target_dependencies(kebni_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  Kebni::sensaition_parser
  kebni_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch parameters
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
