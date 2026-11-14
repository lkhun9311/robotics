#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Records whether each later-stage package resolves in the pinned image.
# AVAILABLE means "apt can resolve it", not "it works".
set -euo pipefail
packages=(
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-mujoco-ros2-control
  ros-jazzy-moveit ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rmw-fastrtps-cpp
  ros-jazzy-lely-core-libraries ros-jazzy-canopen libmodbus-dev
)
apt-get update -qq
for pkg in "${packages[@]}"; do
  if apt-get install --simulate --no-install-recommends -qq "$pkg" >/dev/null 2>&1; then
    echo "AVAILABLE   $pkg"
  else
    echo "UNAVAILABLE $pkg"
  fi
done
