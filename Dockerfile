# Project packaging for the ROS runtime inspected on duck2 (2026-09-08).
# Keep Duckietown's runtime and licensing; install only our application package.
ARG BASE_IMAGE=duckietown/dt-ros-commons:v4.3.0-amd64@sha256:79aa9db043053e8d78e43bd90c7acd02562bee728b69e8512b4d54e3ec8ad858
FROM ${BASE_IMAGE}
ARG BASE_IMAGE
ENV DT_REPO_PATH=/code/catkin_ws/src/duckiebot-ros \
    DT_LAUNCH_PATH=/launch/duckiebot-ros \
    DT_LAUNCHER=default
WORKDIR /code/catkin_ws/src/duckiebot-ros
COPY packages/duckie_lane_follower ./packages/duckie_lane_follower
RUN chmod +x packages/duckie_lane_follower/src/*.py && \
    . /opt/ros/noetic/setup.sh && \
    catkin build --workspace /code/catkin_ws duckie_lane_follower
COPY launchers/ /launch/duckiebot-ros/
RUN chmod +x /launch/duckiebot-ros/*.sh && \
    dt-install-launchers /launch/duckiebot-ros
COPY config/duck2.json ./config/duck2.json
LABEL org.opencontainers.image.title="AlTaTa Duck2 application" \
      org.opencontainers.image.base.name="${BASE_IMAGE}" \
      org.opencontainers.image.description="Project code for duck2's existing ROS Noetic runtime"
# The default command starts no ROS node, subscriber or wheel publisher.
CMD ["bash", "/launch/duckiebot-ros/default.sh"]
