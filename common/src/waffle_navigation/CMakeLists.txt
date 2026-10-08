cmake_minimum_required(VERSION 3.8)
project(waffle_navigation)

find_package(ament_cmake REQUIRED)
find_package(dwb_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
add_library(precision_spin_behavior SHARED src/precision_spin.cpp)
target_compile_features(precision_spin_behavior PUBLIC cxx_std_17)
target_include_directories(precision_spin_behavior PRIVATE include)
ament_target_dependencies(precision_spin_behavior nav2_behaviors nav2_core pluginlib turtlebot3_msgs)
pluginlib_export_plugin_description_file(nav2_core precision_behaviors.xml)
install(TARGETS precision_spin_behavior LIBRARY DESTINATION lib)
add_executable(precision_spin_selftest test/precision_spin_selftest.cpp)
target_compile_features(precision_spin_selftest PUBLIC cxx_std_17)
target_compile_options(precision_spin_selftest PRIVATE -UNDEBUG)
target_include_directories(precision_spin_selftest PRIVATE include)
install(TARGETS precision_spin_selftest DESTINATION lib/${PROJECT_NAME})
add_executable(precision_spin_loadtest test/precision_spin_loadtest.cpp)
target_compile_features(precision_spin_loadtest PUBLIC cxx_std_17)
ament_target_dependencies(precision_spin_loadtest nav2_core pluginlib)
install(TARGETS precision_spin_loadtest DESTINATION lib/${PROJECT_NAME})
add_library(precision_pose_critic SHARED src/precision_pose_critic.cpp src/position_approach_critic.cpp)
target_compile_features(precision_pose_critic PUBLIC cxx_std_17)
target_include_directories(precision_pose_critic PRIVATE include)
ament_target_dependencies(precision_pose_critic dwb_core pluginlib)
pluginlib_export_plugin_description_file(dwb_core precision_plugins.xml)
install(TARGETS precision_pose_critic LIBRARY DESTINATION lib)
add_executable(precision_pose_selftest test/precision_pose_selftest.cpp)
target_compile_features(precision_pose_selftest PUBLIC cxx_std_17)
target_include_directories(precision_pose_selftest PRIVATE include)
ament_target_dependencies(precision_pose_selftest dwb_core pluginlib)
install(TARGETS precision_pose_selftest DESTINATION lib/${PROJECT_NAME})
add_executable(position_approach_selftest test/position_approach_selftest.cpp)
target_compile_features(position_approach_selftest PUBLIC cxx_std_17)
target_include_directories(position_approach_selftest PRIVATE include)
ament_target_dependencies(position_approach_selftest dwb_core pluginlib)
install(TARGETS position_approach_selftest DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/nav2_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_waypoints
)

install(
  PROGRAMS scripts/normalize_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME normalize_scan
)

install(
  PROGRAMS scripts/save_start_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_start_pose
)

install(
  DIRECTORY config launch maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(encoder_departure test/test_encoder_departure.py)
  ament_add_pytest_test(precision_pose_route test/test_precision_pose_route.py)
  ament_add_pytest_test(directional_waypoints test/test_directional_waypoints.py)
  ament_add_pytest_test(way12_continuous test/test_way12_continuous.py)
  ament_add_pytest_test(burger2_port test/test_burger2_port.py)
  ament_add_pytest_test(staged_final test/test_staged_final.py)
  ament_add_pytest_test(position_then_yaw test/test_position_then_yaw.py)
  ament_add_pytest_test(burger2_velocity_samples test/test_burger2_velocity_samples.py)
  ament_add_pytest_test(start_pose test/test_start_pose.py)
  ament_add_pytest_test(arrival_recovery test/test_arrival_recovery.py)
  ament_add_pytest_test(precision_tuning test/test_precision_tuning.py)
  ament_add_pytest_test(overshoot_guard test/test_overshoot_guard.py)
  ament_add_pytest_test(terminal_approach test/test_terminal_approach.py)
endif()

ament_package()
