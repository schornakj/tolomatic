cmake_minimum_required(VERSION 3.5)
project(acsi_eip_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(odva_ethernetip REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tolomatic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS system)

add_library(servo src/servo.cpp)
target_include_directories(servo PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
  ${tolomatic_msgs_INCLUDE_DIRS}  
  ${sensor_msgs_INCLUDE_DIRS}  
  ${odva_ethernetip_INCLUDE_DIRS}  
)
target_link_libraries(servo
  ${rclcpp_LIBRARIES}
  ${Boost_LIBRARIES}  
  ${tolomatic_msgs_LIBRARIES}  
  ${sensor_msgs_LIBRARIES}  
  odva_ethernetip::odva_ethernetip
)

## Declare a cpp executable
add_executable(scanner_node src/scanner_node.cpp)
target_link_libraries(scanner_node
  servo
)

add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node
  servo
)

ament_export_include_directories(
    include)

set(ROS_LIB_DESTINATION lib)
set(ROS_BIN_DESTINATION bin)
set(ROS_INCLUDE_DESTINATION include)
  
## Mark executables and libraries for installation
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
   servo
   servo_node
   scanner_node
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()


