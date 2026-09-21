cmake_minimum_required(VERSION 3.0.2)
project(raicom)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/raicom_auto.py
  scripts/raicom_yolo_trt.py
  scripts/raicom_arm.py
  scripts/raicom_voice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
