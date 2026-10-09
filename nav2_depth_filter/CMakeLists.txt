cmake_minimum_required(VERSION 3.8)
project(nav2_depth_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(depth_filter_component SHARED src/depth_filter_node.cpp)
target_include_directories(depth_filter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(depth_filter_component rclcpp rclcpp_components sensor_msgs)
target_link_libraries(depth_filter_component ${OpenCV_LIBS})

rclcpp_components_register_nodes(depth_filter_component "nav2_depth_filter::DepthFilterNode")

install(TARGETS depth_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_filter test/test_depth_filter.cpp)
  target_include_directories(test_depth_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_depth_filter ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_export_libraries(depth_filter_component)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs OpenCV)
ament_package()
