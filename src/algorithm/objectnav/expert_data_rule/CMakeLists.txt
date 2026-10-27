cmake_minimum_required(VERSION 3.0.2)
project(expert_data_rule)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  data_generate
  geometry_msgs
  message_filters
  nav_msgs
  robot_control
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  stonefish_ros
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  cv_bridge data_generate geometry_msgs message_filters nav_msgs robot_control rospy
  sensor_msgs std_msgs std_srvs stonefish_ros
)

catkin_install_python(PROGRAMS
  scripts/rule_expert_node.py
  scripts/perception_worker_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
