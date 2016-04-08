cmake_minimum_required(VERSION 2.8.3)
project(suivi)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy md49test)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} tracker/include)

add_executable(talker controleur/src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker suivi_generate_messages_cpp)

add_executable(listener controleur/src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener suivi_generate_messages_cpp)

#add_executable(tracker tracker/src/tracker.cpp tracker/src/Application.cpp tracker/src/ColorVideoStream.cpp tracker/src/DepthVideoStream.cpp #tracker/src/FeatureCapture.cpp tracker/src/LowPassFilter.cpp tracker/src/VideoStream.cpp)
#target_link_libraries(tracker ${catkin_LIBRARIES})
#add_dependencies(tracker suivi_generate_messages_cpp)
