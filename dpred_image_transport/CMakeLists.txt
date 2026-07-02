cmake_minimum_required(VERSION 3.16)
project(dpred_image_transport)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(depth_codec REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dpred_publisher.cpp
  src/dpred_subscriber.cpp
  src/manifest.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  depth_codec::depthcodec
  image_transport::image_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS})

pluginlib_export_plugin_description_file(image_transport dpred_plugins.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(depth_codec image_transport pluginlib rclcpp sensor_msgs)
ament_package()
