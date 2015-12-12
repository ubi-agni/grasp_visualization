cmake_minimum_required(VERSION 2.8.3)
project(rqt_grasp_viewer)

find_package(catkin REQUIRED COMPONENTS grasp_viewer)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
   scripts/rqt_grasp_viewer
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ui
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


