cmake_minimum_required(VERSION 3.5)
project(aeb_gazebo)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── CAN message types ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AebRadarTarget.msg"
  "msg/AebEgoVehicle.msg"
  "msg/AebBrakeCmd.msg"
  "msg/AebFsmState.msg"
  "msg/AebAlert.msg"
  DEPENDENCIES std_msgs
)

# ── Python nodes ─────────────────────────────────────────────────────────────
# AEB control logic runs on the Zephyr ECU (aeb-ecu container, native_sim);
# no Python or C++ controller node is launched from this package.
install(PROGRAMS
  src/scenario_controller.py
  src/target_vehicle_controller.py
  src/dashboard_node.py
  src/dashboard_overlay.py
  src/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Resource files ────────────────────────────────────────────────────────────
install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
