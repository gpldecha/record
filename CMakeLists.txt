cmake_minimum_required(VERSION 2.8.3)
project(record)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp tf std_msgs message_generation optitrack_rviz rosbag visualization_msgs)

add_service_files(
  FILES
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime rosbag
)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

set(MOUSE_INCLUDE /home/guillaume/CppWorkSpace/mouse/include              )
set(MOUSE_LIB       /home/guillaume/CppWorkSpace/mouse/lib/libmouse.so    )

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${path_mouse}/include
                    ${catkin_INCLUDE_DIRS}
)

add_library(record ${INCLUDE_DIR}/record/record.h  ${SRC_DIR}/record.cpp )
target_link_libraries(record ${catkin_LIBRARIES}                         )

# RECORD POSITION

add_library(record_position ${INCLUDE_DIR}/record/record_position.h ${SRC_DIR}/record_position.cpp)
target_link_libraries(record_position record)

add_executable(record_position_node ${SRC_DIR}/record_position_node.cpp)
target_link_libraries(record_position_node record_position)

#RECORD FORCE-TORQUE

add_library(record_ft ${INCLUDE_DIR}/record/record_forcetorque.h ${SRC_DIR}/record_forcetorque.cpp)
target_link_libraries(record_ft record)

add_executable(record_force_torque_node ${SRC_DIR}/record_force_torque_node.cpp)
target_link_libraries(record_force_torque_node record_ft)


#RECORD POSITION and FORCE-TORQUE

add_executable(record_pos_ft_node ${SRC_DIR}/record_pos_ft.cpp)
target_link_libraries(record_pos_ft_node record_ft record_position)


#MOUSE LISTENER NODE
include_directories(${MOUSE_INCLUDE})
add_executable(mouse_node ${SRC_DIR}/mouse_node.cpp)
target_link_libraries(mouse_node ${catkin_LIBRARIES} record ${MOUSE_LIB})




