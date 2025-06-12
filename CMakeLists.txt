cmake_minimum_required(VERSION 3.0.2)
project(mic_to_whisper)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/mic_node.py
  src/whisper_node.py
  src/transcription_viewer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
