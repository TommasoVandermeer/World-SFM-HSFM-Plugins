cmake_minimum_required(VERSION 3.8)
project(world_sfm_hsfm_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(WorldSFMPlugin SHARED src/WorldSFMPlugin.cpp)
target_include_directories(WorldSFMPlugin PUBLIC include)
ament_target_dependencies(WorldSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
)
ament_export_libraries(WorldSFMPlugin)
target_link_libraries(WorldSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

set(msg_files
  "msg/Vector2.msg"
  "msg/Pose2.msg"
  "msg/Forces.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(WorldSFMPlugin "${cpp_typesupport_target}")

install(TARGETS WorldSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY models/
   DESTINATION share/${PROJECT_NAME}/models
)
install(DIRECTORY config/
   DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/spawn_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(load_agent_params scripts/load_agent_params.cpp)
ament_target_dependencies(load_agent_params rclcpp rcl_interfaces)

install(TARGETS
  load_agent_params
  DESTINATION lib/${PROJECT_NAME})

ament_package()
