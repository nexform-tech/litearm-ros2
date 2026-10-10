cmake_minimum_required(VERSION 3.16)
project(litearm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointParam.msg"
  "msg/LitearmStatus.msg"

  "srv/GetFeedforwardScalar.srv"
  "srv/GetJointParams.srv"
  "srv/GetStatus.srv"
  "srv/SetFeedforwardMask.srv"
  "srv/SetFeedforwardPreset.srv"
  "srv/SetFeedforwardScalar.srv"
  "srv/SetFeedforwardVector.srv"
  "srv/SetJointGains.srv"
  "srv/SetJointLimits.srv"
  "srv/SetMotionMode.srv"
  "srv/SetPayload.srv"
  "srv/SetSpeedScaling.srv"
  "srv/GetTcp.srv"
  "srv/GetDiagnostics.srv"
  "srv/KinBench.srv"
  "srv/MoveJ.srv"
  "srv/MoveJSync.srv"
  "srv/MoveP.srv"
  "srv/MoveJs.srv"
  "srv/SendMit.srv"
  "srv/SendMitAll.srv"
  "srv/MoveL.srv"
  "srv/MoveC.srv"
  "srv/MovePath.srv"
  "srv/PollCart.srv"
  "srv/InverseKinematics.srv"
  "srv/GetFeedforwardVector.srv"
  "srv/GetFeedforwardMask.srv"
  "srv/GetFeedforwardCatalog.srv"
  "srv/SetGravityScale.srv"
  "srv/SetInertiaScale.srv"
  "srv/SetGravityVector.srv"
  "srv/ProbeModel.srv"
  "srv/GetModelBody.srv"
  "srv/SetModelBody.srv"
  "srv/GetModelJm.srv"
  "srv/SetModelJm.srv"
  "srv/CommitModel.srv"
  "srv/GetModelStatus.srv"
  "srv/LogStart.srv"
  "srv/LogDump.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
