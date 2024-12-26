cmake_minimum_required(VERSION 3.8)
project(agrobot_interfaces)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MotorPWMs.msg
  msg/EncoderPulses.msg
  msg/ServoAngles.msg
  srv/ServiceExample.srv
  action/ActionExample.action
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
