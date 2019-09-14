cmake_minimum_required(VERSION 2.8.3)
project(agent)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	genmsg
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(DEPENDENCIES std_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} include/agent)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs 
)

add_executable(cs src/cs.cpp )
target_link_libraries(cs ${catkin_LIBRARIES} pthread rt)
add_dependencies(cs agent_generate_messages_cpp)

add_executable(ve src/ve.cpp )
target_link_libraries(ve ${catkin_LIBRARIES} pthread rt)
add_dependencies(ve agent_generate_messages_cpp)


