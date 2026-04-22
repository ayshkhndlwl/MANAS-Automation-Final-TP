cmake_minimum_required(VERSION 3.8)
project(task2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Action Generators
find_package(rosidl_default_generators REQUIRED)

# 1. Generate the Custom Action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Nav2Pose.action"
)

# 2. Install Python Executables directly
# NOTE: This assumes your python files are in a folder called 'scripts'. 
# If they are in a folder called 'task2', change the word 'scripts' below to 'task2'!
install(PROGRAMS
  task2/controller.py
  task2/dead_reckoning.py
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install physical assets, worlds, and launch files
install(DIRECTORY
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()