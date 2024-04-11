find_package(dynamixel_sdk REQUIRED)
add_library(motor_driver SHARED motor_driver.cpp)

target_include_directories(motor_driver
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


add_library(diffdrive_turtlebot3 SHARED diffdrive_turtlebot3.cpp wheel.cpp)

target_include_directories(diffdrive_turtlebot3
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(diffdrive_turtlebot3
  PUBLIC
    hardware_interface
    pluginlib
    rclcpp
)

target_link_libraries(diffdrive_turtlebot3 PUBLIC motor_driver)

install(
  TARGETS diffdrive_turtlebot3 motor_driver
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(diffdrive_turtlebot3)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
