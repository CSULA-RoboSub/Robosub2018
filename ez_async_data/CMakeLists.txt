cmake_minimum_required(VERSION 2.8.4)
project(ez_async_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Rotation.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

add_subdirectory(cpp libvncxx)

if(CMAKE_COMPILER_ID MATCHES "GNU|Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

include_directories(
  cpp/include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCE_FILES
    main.cpp)

add_executable(ez_async_data ${SOURCE_FILES})

target_link_libraries(ez_async_data PRIVATE libvncxx)
target_link_libraries(ez_async_data
   ${catkin_LIBRARIES}
)

if (UNIX OR APPLE)
	target_link_libraries(ez_async_data LINK_PUBLIC pthread)
else()
	target_link_libraries(ez_async_data LINK_PUBLIC Advapi32 SetupAPI)
endif()
