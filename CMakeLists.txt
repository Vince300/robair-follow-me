cmake_minimum_required(VERSION 2.8.3)
project(suivi)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy md49test)
find_package( OpenCV REQUIRED )
FIND_LIBRARY( OPENNI2_LIBRARY
             NAMES OpenNI2
             HINTS /home/robair/groupeDufour/OpenNI-Linux-x86-2.2
             PATH_SUFFIXES lib lib64
)
FIND_PATH( OPENNI2_INCLUDE_DIR OpenNI.h
          HINTS /home/robair/groupeDufour/OpenNI-Linux-x86-2.2
          PATH_SUFFIXES openni2 ni2)

# Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} tracker/include)

add_executable(talker controleur/src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker suivi_generate_messages_cpp)

add_executable(listener controleur/src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener suivi_generate_messages_cpp)


install(DIRECTORY launch/ 
DESTINATION $(CATKIN_PACKAGE_SHARE_DESTINATION)/launch)

add_executable(tracker tracker/src/tracker.cpp tracker/src/Application.cpp tracker/src/ColorVideoStream.cpp tracker/src/DepthVideoStream.cpp tracker/src/FeatureCapture.cpp tracker/src/LowPassFilter.cpp tracker/src/VideoStream.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES}  ${OPENNI2_LIBRARY} opencv_core opencv_highgui opencv_imgproc)
link_directories( ${OPENNI2_LIBRARY} ${OpenCV_LIBS})
INCLUDE_DIRECTORIES( ${OPENNI2_INCLUDE_DIR} )
add_dependencies(tracker suivi_generate_messages_cpp)
