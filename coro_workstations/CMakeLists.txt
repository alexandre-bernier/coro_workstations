cmake_minimum_required(VERSION 3.1.3)
project(coro_workstations)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/test_move_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/copy_file.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
