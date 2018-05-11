cmake_minimum_required(VERSION 2.8.3)
project(microstrain)
find_package(catkin REQUIRED COMPONENTS
  bot_core
  bot_param
  concise_args
)

catkin_package(
  LIBRARIES image_utils image_utils_renderer
  CATKIN_DEPENDS bot_core
  DEPENDS GLIB2
)


add_executable(microstrain ../src/microstrain_comm.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(microstrain ${catkin_LIBRARIES})


add_executable(microstrain_pose ../src/microstrain_pose.cpp)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(microstrain_pose ${catkin_LIBRARIES})
