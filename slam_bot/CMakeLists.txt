cmake_minimum_required(VERSION 3.5)
project(slam_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(lpigpiod_if2 REQUIRED)
# find_package(chrono REQUIRED)

INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

target_include_directories(/usr/local/include/libpigpiod_if2)

add_executable(encs_pubs_exec src/enc_publisher.cpp)
ament_target_dependencies(encs_pubs_exec rclcpp std_msgs)

add_executable(teleop_drive_exe src/cmd_vel_to_drive.cpp)
ament_target_dependencies(teleop_drive_exe rclcpp geometry_msgs libpigpiod_if2.so)



install(TARGETS
  encs_pubs_exec
  teleop_drive_exe
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
