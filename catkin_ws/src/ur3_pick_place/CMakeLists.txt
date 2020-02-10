cmake_minimum_required(VERSION 2.8.3)
project(ur3_pick_place)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  geometry_msgs
)

# Makes src/ur3_pick_place/ (gripper_interface.py) importable, shared by
# both scripts/static/ and scripts/dynamic/.
catkin_python_setup()

add_message_files(
  FILES
  DetectedObject.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/static/static_pick_place.py
  scripts/dynamic/camera_calibration.py
  scripts/dynamic/pointcloud_object_detector.py
  scripts/dynamic/dynamic_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY env/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/env
  USE_SOURCE_PERMISSIONS
)
