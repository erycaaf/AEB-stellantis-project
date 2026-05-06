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

# NOTE: C++ controller node disabled for Docker build.
# The Python aeb_node.py provides equivalent functionality.
# To re-enable, set AEB_C_ROOT to the embedded C source tree
# and uncomment the add_executable section below.
#
# set(AEB_C_ROOT "/path/to/c_embedded" CACHE PATH "AEB C sources")
# add_executable(aeb_controller_node src/aeb_controller_node.cpp ...)
# ...

# ── Python nodes ─────────────────────────────────────────────────────────────
install(PROGRAMS
  src/aeb_node.py
  src/scenario_controller.py
  src/target_vehicle_controller.py
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
