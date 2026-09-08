cmake_minimum_required(VERSION 3.16)
project(duck2_autonomy)
find_package(ament_cmake REQUIRED)
install(PROGRAMS autonomy.py supervisor.py gateway.py DESTINATION lib/${PROJECT_NAME})
install(FILES runtime.py safety.py lifecycle.py
  ../packages/duckie_lane_follower/src/duck2_core.py
  ../packages/duckie_lane_follower/src/command_gateway.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES diagnostic.yaml diagnostic.launch.py DESTINATION share/${PROJECT_NAME})
ament_package()
