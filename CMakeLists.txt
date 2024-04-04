cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
#action
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)

#EXECUTABLES
#action-server
add_executable(tortoisebot_action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2 tf2_ros)
rosidl_target_interfaces(tortoisebot_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
#action-client
add_executable(exp_action_client src/exp_action_client.cpp)
ament_target_dependencies(exp_action_client rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2 tf2_ros)
rosidl_target_interfaces(exp_action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

#install executables
install(TARGETS
  tortoisebot_action_server
  exp_action_client
  DESTINATION lib/${PROJECT_NAME}
)

#folders to install
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/test_action_client.cpp
  )

  # Add a gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  # Set the timeout for MyTest to X seconds
  set_tests_properties(${PROJECT_NAME}_test PROPERTIES
  	TIMEOUT 63
	  # PASS_REGULAR_EXPRESSION "[  PASSED  ]"
    # FAIL_REGULAR_EXPRESSION "FAILED  ] 1 test,"
	  # ENVIRONMENT "VAR1=value1;VAR2=value2"
  )
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs tf2 tf2_ros)
  rosidl_target_interfaces(${PROJECT_NAME}_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # install the test executables
  install(TARGETS
    ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
