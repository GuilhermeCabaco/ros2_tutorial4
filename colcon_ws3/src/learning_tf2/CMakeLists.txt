cmake_minimum_required(VERSION 3.5)
project(learning_tf2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    turtlesim
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster ${dependencies})

add_executable(turtle_tf2_listener_simple src/turtle_tf2_listener_simple.cpp)
ament_target_dependencies(turtle_tf2_listener_simple ${dependencies})

install(TARGETS
  turtle_tf2_broadcaster
  turtle_tf2_listener_simple
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
