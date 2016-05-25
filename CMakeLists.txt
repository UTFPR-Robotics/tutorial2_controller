cmake_minimum_required(VERSION 2.8.3)
project(tutorial2_controller)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  roslaunch
)

roslaunch_add_file_check(launch)

add_action_files(
   DIRECTORY action
   FILES FuzzyControl.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs   std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(FuzzyServer src/FuzzyServer.cpp)

target_link_libraries(FuzzyServer ${catkin_LIBRARIES} libfuzzylite.so)

add_dependencies(FuzzyServer ${tutorial2_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(FuzzyClient src/FuzzyClient.cpp)

target_link_libraries(FuzzyClient ${catkin_LIBRARIES})

add_dependencies(FuzzyClient ${tutorial2_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
