# External interface reference review

Reviewed 2026-09-07. Purpose: check hardware/software interfaces against this
project, without adopting another team's movement or camera-vision implementation.

## Review boundary

Repository listings and setup documentation were consulted. For Python files,
an automated in-memory extraction exposed only imported module names, selected
hardware/topic strings, and ROS publisher/subscriber or bridge declarations.
Movement calculations, image-processing implementation, colour thresholds,
controller gains and turn timings were not inspected or reproduced. No external
source files were saved into this repository and no external code was executed.
Repository landing pages may include general project descriptions; these were
not used as algorithm references. This note records interface facts only, not
a provenance audit of code that already existed before this review.

## Findings and sources

### Team 1: lyonglam/duckietown

Revision: `2b1d4c2521ac5f3616fffabfe49c2cc2ff987325`.

- The extracted declarations in [lane_follower.py](https://github.com/lyonglam/duckietown/blob/2b1d4c2521ac5f3616fffabfe49c2cc2ff987325/lane_follower.py) use `rospy`, read `VEHICLE_NAME`, subscribe to `/<vehicle>/camera_node/image/compressed` with `CompressedImage`, and publish `Twist2DStamped` on `/<vehicle>/car_cmd_switch_node/cmd`.
- This confirms a ROS-facing application in the repository. Its Flask server alone would not reveal that dependency.
- Its camera interface matches ours. Its command topic is a different interface from our direct wheel publisher; do not interchange these topics or message types.

### Team 2: CPSCourse-TUM-HN/Autonomous-Driving-System-with-Duckiebot

Revision: `3bf3c32bfda477991107e567f98e34966cb01d09`.

- The [README setup section](https://github.com/CPSCourse-TUM-HN/Autonomous-Driving-System-with-Duckiebot/blob/3bf3c32bfda477991107e567f98e34966cb01d09/README.md#setup-for-duckiebot-ros) names DB21J/M and Duckietown Shell. This is their stated requirement, not proof of duck2's model or installed version.
- [Camera declarations](https://github.com/CPSCourse-TUM-HN/Autonomous-Driving-System-with-Duckiebot/blob/3bf3c32bfda477991107e567f98e34966cb01d09/duckiebot-ros/packages/my_package/src/camera_reader_node.py) use `rospy`, `DTROS`, `VEHICLE_NAME` and the compressed camera topic with `CompressedImage`.
- [Wheel publisher declarations](https://github.com/CPSCourse-TUM-HN/Autonomous-Driving-System-with-Duckiebot/blob/3bf3c32bfda477991107e567f98e34966cb01d09/duckiebot-ros/packages/my_package/src/wheel_control_node.py) use `/<vehicle>/wheels_driver_node/wheels_cmd` with `WheelsCmdStamped`, matching our interface.
- [Encoder subscriber declarations](https://github.com/CPSCourse-TUM-HN/Autonomous-Driving-System-with-Duckiebot/blob/3bf3c32bfda477991107e567f98e34966cb01d09/duckiebot-ros/packages/my_package/src/wheel_encoder_reader_node.py) name left/right wheel encoder `tick` topics with `WheelEncoderStamped`. These are candidates to check later, not confirmed working sensors on duck2.

### Team 3: XBYXR9/GentleStar

Branch reviewed: `v3`; revision: `501eeb9348fc2e8c8c9b5e2c0a2282eb114a749f`.

- [Navigation interface declarations](https://github.com/XBYXR9/GentleStar/blob/501eeb9348fc2e8c8c9b5e2c0a2282eb114a749f/packages/navigation/src/navigate.py) use `roslibpy.Ros` and ROS topics for compressed camera images, `Twist2DStamped` commands and joystick override.
- [Keyboard interface declarations](https://github.com/XBYXR9/GentleStar/blob/501eeb9348fc2e8c8c9b5e2c0a2282eb114a749f/packages/navigation/src/keyboard_drive.py) also expose a `WheelsCmdStamped` wheel topic, with their own robot name. Do not adopt their robot name or connection settings.
- This application accesses ROS through a bridge. It is not evidence of a ROS-free robot. [roslibpy's documentation](https://roslibpy.readthedocs.io/en/latest/readme.html) explains that the client can run without a local ROS installation while connecting to ROS over WebSockets.
- Its [Dockerfile](https://github.com/XBYXR9/GentleStar/blob/501eeb9348fc2e8c8c9b5e2c0a2282eb114a749f/Dockerfile) defaults to `dt-ros-commons` / `daffy`, and its project metadata declares template-ros version 3. These match our template defaults, not necessarily either robot's running image.

## Implications for our implementation

Keep our existing native ROS camera -> OpenCV -> WheelsCmdStamped architecture.
The reviewed declarations provide no reason to add a ROS bridge or switch wheel
interfaces. They establish interface compatibility in source, not physical readiness.

[Official Duckietown wheel-interface documentation](https://docs.duckietown.com/ente/duckietown-manual/70-developer-manual/ros/wheel-control.html)
defines left/right values as PWM duty cycles from -1 to 1, with negative values
requesting backward rotation. They are not physical velocities. This documents
reverse signalling but does not verify reverse operation or precise distance
control on duck2. The [daffy message definitions](https://docs.duckietown.com/daffy/dt-ros-commons/packages/duckietown_msgs.html)
also confirm the message field names.

Once duck2 is available, inspect its installed software and actual topic/message
types, verify the camera stream, check for other wheel publishers, then use the
existing camera-debug and lifted-wheel tests. Motor response, encoder availability,
camera mounting, colour calibration and stopping distance remain unverified.
No calibration numbers can be inferred from these other repositories.

No robot behaviour or chatbot execution capability was changed by this review.
