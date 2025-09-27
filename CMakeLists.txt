cmake_minimum_required(VERSION 3.5)
project(vacuum_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_FOR_ROBOT "Build minimal for robot" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
if(NOT BUILD_FOR_ROBOT)
  find_package(nav2_msgs REQUIRED)
  find_package(nav2_core REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(Fields2Cover REQUIRED)
  find_package(opennav_coverage_msgs REQUIRED)
endif()
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(PAHO_MQTT_CPP REQUIRED paho-mqttpp3)
# pkg_check_modules(PAHO_MQTT_C REQUIRED paho-mqtt3as)

include_directories(/usr/include)
link_directories(/usr/local/lib)
link_directories(/usr/lib/x86_64-linux-gnu)

include_directories(
  include 
  /usr/local/include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CoverageZone.msg"
  "msg/CoveragePlan.msg"
  "msg/CoverageMission.msg"
  DEPENDENCIES geometry_msgs nav_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

# Executables
if(NOT BUILD_FOR_ROBOT)
  add_executable(sim_step_controller_node
      src/sim_step_controller_node.cpp
      src/easywsclient.cpp
  )
  add_executable(web_bridge_node
      src/web_bridge_node.cpp
      src/easywsclient.cpp
  )
  add_executable(coverage_nav2_node 
      src/coverage_nav2_node.cpp
  )
endif()

if(NOT BUILD_FOR_ROBOT)
  add_executable(mqtt_ros2_bridge
      src/mqtt_ros2_bridge.cpp
  )
endif()

add_executable(debug_node 
    src/debug_node.cpp
)

if(BUILD_FOR_ROBOT)
  add_executable(real_step_controller_node
      src/real_step_controller_node.cpp
  )

  target_compile_definitions(real_step_controller_node PRIVATE BUILD_FOR_ROBOT)

  target_link_libraries(real_step_controller_node
    pigpiod_if2   
    pigpio       
    rt
    pthread
  )
endif()


# Dependencies
if(NOT BUILD_FOR_ROBOT)
  ament_target_dependencies(sim_step_controller_node
    rclcpp
    geometry_msgs
    nav_msgs
    nav2_core
    pluginlib
    tf2_ros
  )

  ament_target_dependencies(web_bridge_node
    rclcpp
    geometry_msgs
    nav_msgs
  )

  ament_target_dependencies(coverage_nav2_node
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
    nav2_core
    pluginlib
    Fields2Cover
    lifecycle_msgs
    opennav_coverage_msgs
  )

  ament_target_dependencies(mqtt_ros2_bridge
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
  )
endif()


ament_target_dependencies(debug_node rclcpp 
    std_msgs
)


if(NOT BUILD_FOR_ROBOT)
  target_link_libraries(mqtt_ros2_bridge
    paho-mqttpp3
    paho-mqtt3a
  )
  rosidl_target_interfaces(mqtt_ros2_bridge
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

  rosidl_target_interfaces(coverage_nav2_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()


if(BUILD_FOR_ROBOT)

  ament_target_dependencies(real_step_controller_node
    rclcpp
    geometry_msgs
    nav_msgs
    tf2_ros
  )

endif()

# Plugin export
if(NOT BUILD_FOR_ROBOT)
  pluginlib_export_plugin_description_file(nav2_core plugin.xml)
endif()

# Install targets
if(NOT BUILD_FOR_ROBOT)
  install(TARGETS
    sim_step_controller_node
    web_bridge_node
    coverage_nav2_node
    mqtt_ros2_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES plugin.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install other files
if(BUILD_FOR_ROBOT)
  install(TARGETS
    real_step_controller_node
    # mqtt_ros2_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS
  debug_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
