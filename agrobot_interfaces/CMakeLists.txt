cmake_minimum_required(VERSION 3.8)
project(agrobot_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MotorPWMs.msg
  msg/EncoderPulses.msg
  msg/WheelAngularVel.msg
  msg/ServoAngles.msg
  msg/PIDWheelError.msg
  msg/SensorDatas.msg
  msg/LimitSwitchStates.msg
  srv/ServiceExample.srv
  action/ActionExample.action
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
