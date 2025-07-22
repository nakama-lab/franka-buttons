###########################################
# Dockerfile Summary
###########################################
# Purpose: Build a ROS2 Humble container with the Franka Buttons package
#
# Major Components:
# - Base image: osrf/ros:humble-desktop
# - Copies the franka_buttons/ and franka_buttons_interfaces/ folders into the Docker image to build
###########################################

# Use full ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop

# Execute all commands with bash instead of sh (the default)
SHELL ["/bin/bash", "-c"]

# Install APT dependencies as cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        # Add any packages to install below this line
        python3-pip \
        python3-venv

# Prepare an overlay workspace
ENV OVERLAY_WS=/franka_buttons_ws

# Prepare virtual environment
WORKDIR $OVERLAY_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && python3 -m venv .venv --system-site-packages --symlinks \
    && touch .venv/COLCON_IGNORE

# Copy the Franka Buttons source
WORKDIR $OVERLAY_WS/src
COPY franka_buttons/ franka_buttons/
COPY franka_buttons_interfaces/ franka_buttons_interfaces/

# Install all missing Python and ROS 2 dependencies as specified in their requirements.txt files
WORKDIR $OVERLAY_WS
RUN source .venv/bin/activate \
    && find ./src -name "requirement*.txt" -type f -exec pip3 install -r '{}' ';'
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y 

# Build all packages with the virtual environment sourced
WORKDIR $OVERLAY_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source .venv/bin/activate \
    # If more packages need the virtual environment build, add them at the end of the following line
    && python -m colcon build --symlink-install --packages-up-to \
        franka_buttons

# Extend the default ROS entrypoint script so that the new workspace is also sourced by default
RUN sudo sed --in-place \
    --expression '$isource "$OVERLAY_WS/install/setup.bash" --' \
    /ros_entrypoint.sh
