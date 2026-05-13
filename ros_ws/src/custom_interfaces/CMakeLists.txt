cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/MotorState.msg"
  "msg/MotorFault.msg"
  "msg/OperationCommand.msg"
  "msg/PositionPPCommand.msg"
  "msg/VelocityCommand.msg"
  "msg/CurrentCommand.msg"
  "msg/PositionCSPCommand.msg"
  # Services
  "srv/EnableMotor.srv"
  "srv/SetRunMode.srv"
  "srv/SetZeroPosition.srv"
  "srv/ReadParam.srv"
  "srv/WriteParam.srv"
  "srv/Help.srv"
  "srv/GetCanConfig.srv"
  "srv/SetCanConfig.srv"
  "srv/SetActiveReport.srv"
  "srv/MotorParam.srv"
  "srv/SetMimicMode.srv"
  "srv/SetMimicParams.srv"
  "srv/EnableMimicMotors.srv"
  # Actions
  "action/MoveToPosition.action"
  "action/SetVelocity.action"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
    action_msgs
)

ament_package()
