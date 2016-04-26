cmake_minimum_required(VERSION 2.8.3)
project(suivi)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy md49test std_msgs message_generation)
find_package( OpenCV REQUIRED )

# INclude directories
include_directories(include ${catkin_INCLUDE_DIRS} tracker/include $ENV{OPENNI2_INCLUDE})
link_directories( $ENV{OPENNI2_REDIST} ${OpenCV_LIBS})

add_message_files(FILES image.msg)
generate_messages(DEPENDENCIES std_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(talker controleur/src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker suivi_generate_messages_cpp)
 
add_executable(listener controleur/src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener suivi_generate_messages_cpp)


install(DIRECTORY launch/ 
DESTINATION $(CATKIN_PACKAGE_SHARE_DESTINATION)/launch)

add_executable(tracker 
    tracker/src/tracker.cpp
    tracker/src/Application.cpp
    tracker/src/ColorVideoStream.cpp
    tracker/src/DepthVideoStream.cpp
    tracker/src/FeatureCapture.cpp
    tracker/src/LowPassFilter.cpp
    tracker/src/MotorDriver.cpp
    tracker/src/VideoStream.cpp
    tracker/src/ObstaclesBasicTracker.cpp
    tracker/src/SlidingWindowTracker.cpp
    tracker/src/TargetTracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES}  OpenNI2 opencv_core opencv_highgui opencv_imgproc)
add_dependencies(tracker suivi_generate_messages_cpp)
