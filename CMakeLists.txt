cmake_minimum_required(VERSION 3.1.3)
project(tocabi_w_syhand_moveit_config)

find_package(catkin REQUIRED
  rospy
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
