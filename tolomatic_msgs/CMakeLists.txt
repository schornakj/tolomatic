cmake_minimum_required(VERSION 3.5)
project(tolomatic_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Boost 1.58 REQUIRED COMPONENTS system)

set(msg_files
    "msg/AcsiInputs.msg"
    "msg/AcsiOutputs.msg"
    "msg/AcsiStatus.msg"
    )

set(srv_files
    "srv/AcsiSetProfile.srv"
    "srv/AcsiMoveAbsolute.srv"
    "srv/AcsiMoveIncremental.srv"
    "srv/AcsiMoveSelect.srv"
    "srv/AcsiMoveRotary.srv"
    "srv/AcsiMoveVelocity.srv"
    )
  
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs std_srvs)
  
ament_export_dependencies(rosidl_default_runtime)

ament_package()
