cmake_minimum_required(VERSION 3.0.2)
project(raicom)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  Detection2D.msg
  DetectionArray.msg
  ParkTarget.msg
  TaskState.msg
)

add_service_files(
  FILES
  SavePose.srv
)

add_action_files(
  FILES
  AlignTarget.action
  ArmTask.action
  RunMission.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    message_runtime
)

catkin_install_python(PROGRAMS
  scripts/raicom_auto.py
  scripts/raicom_yolo_trt.py
  scripts/raicom_arm.py
  scripts/raicom_voice.py
  scripts/raicom_alignment_node.py
  scripts/raicom_arm_node.py
  scripts/raicom_mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
