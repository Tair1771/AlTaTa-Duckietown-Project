ARG ROS2_BASE=ros:jazzy-ros-base@sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4
FROM ${ROS2_BASE}
RUN apt-get update && apt-get install -y --no-install-recommends python3-opencv python3-numpy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /project
COPY packages/duckie_lane_follower/src/ /project/packages/duckie_lane_follower/src/
COPY ros2/ /project/ros2/
COPY tests/ /project/tests/
COPY tools/ /project/tools/
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths ros2
ENV PYTHONPATH=/project/packages/duckie_lane_follower/src:/project/ros2
ENV VEHICLE_NAME=duck2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
CMD ["bash", "-c", "source /project/install/setup.bash && ros2 launch duck2_autonomy diagnostic.launch.py"]
