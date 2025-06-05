cmake_minimum_required(VERSION 3.0.2)
project(mic_to_whisper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mic_node.py
  scripts/whisper_node.py
  scripts/transcription_viewer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
