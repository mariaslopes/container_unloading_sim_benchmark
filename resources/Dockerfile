FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LC_NUMERIC=en_US.UTF-8

WORKDIR /home/ws_unloading
COPY . src/container_unloading_sim_benchmark
RUN vcs import --skip-existing src < src/container_unloading_sim_benchmark/resources/dependencies.repos && \
    rm -rf src/shared/rox/rox_navigation \
    src/rox/rox_bringup \
    src/rox/rox_rviz \
    src/rox/rox_navigation \
    src/ros2_epick_gripper/epick_controllers \
    src/ros2_epick_gripper/epick_driver \
    src/ros2_epick_gripper/epick_hardware_tests \
    src/ros2_epick_gripper/epick_moveit_plugin \
    src/ros2_epick_gripper/epick_moveit_studio \
    src/ros2_epick_gripper/epick_msgs && \
    apt-get update && \
    ldconfig && \
    rosdep install --from-paths src --ignore-src -r -y && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install && \
    mv src/container_unloading_sim_benchmark/resources/ros_entrypoint.sh /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]