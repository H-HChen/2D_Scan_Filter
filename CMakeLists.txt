cmake_minimum_required(VERSION 3.5)
project(scan_filter)

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
find_package(geometry_msgs      REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(serial             REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(laser_filters      REQUIRED)
find_package(laser_geometry     REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include include/scan_filter/
)

set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "laser_filters"
  "laser_geometry"
)

set(EXECUTABLE_NAME "filter_node")

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_executable(${EXECUTABLE_NAME} 
  src/filter.cpp)

ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})


ament_package()
