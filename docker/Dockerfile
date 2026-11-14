# syntax=docker/dockerfile:1
# SPDX-License-Identifier: Apache-2.0
ARG ROS_IMAGE
# Default keeps every stage's FROM resolvable when only the dev target is built (BuildKit
# resolves all stages); release builds pass the pinned digest from .env.
ARG RUNTIME_IMAGE=ros:jazzy-ros-core
FROM ${ROS_IMAGE} AS base
ARG ROS_SNAPSHOT
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# Pin ROS packages to a dated, signed snapshot. The key file must hold exactly one primary key
# with the expected fingerprint, so a swapped key, or a second key appended to the file (apt
# would trust every key in the keyring), fails the build instead of changing the trust root.
COPY docker/keys/ros-snapshots.asc /tmp/ros-snapshots.asc
RUN gpg --dearmor < /tmp/ros-snapshots.asc > /usr/share/keyrings/ros-snapshots.gpg \
 && keys="$(gpg --show-keys --with-colons /usr/share/keyrings/ros-snapshots.gpg)" \
 && primaries="$(grep -c '^pub:' <<< "$keys" || true)" \
 && if [ "$primaries" != 1 ]; then \
      echo "ros snapshot key file holds ${primaries:-0} primary keys, expected exactly 1" >&2; exit 1; fi \
 && fpr="$(awk -F: '/^fpr:/{print $10; exit}' <<< "$keys")" \
 && if [ "$fpr" != "4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA" ]; then \
      echo "ros snapshot key fingerprint mismatch: ${fpr:-none}" >&2; exit 1; fi \
 && rm /tmp/ros-snapshots.asc \
 && : "${ROS_SNAPSHOT:?set ROS_SNAPSHOT}" \
 && rm -f /etc/apt/sources.list.d/ros2.sources \
 && printf '%s\n' 'Types: deb' \
      "URIs: http://snapshots.ros.org/jazzy/${ROS_SNAPSHOT}/ubuntu" \
      'Suites: noble' 'Components: main' \
      'Signed-By: /usr/share/keyrings/ros-snapshots.gpg' > /etc/apt/sources.list.d/ros2.sources
RUN apt-get update \
 && if grep -Rqs 'packages\.ros\.org' /etc/apt/sources.list /etc/apt/sources.list.d; then \
      echo 'an apt source still points at packages.ros.org' >&2; exit 1; fi \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

FROM base AS dev
ARG MENAGERIE_COMMIT
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libgtest-dev clang-format clang-tidy gdb valgrind gcovr \
      python3-pytest can-utils iproute2 rt-tests util-linux kmod \
      ros-jazzy-mujoco-ros2-control ros-jazzy-joint-trajectory-controller \
      ros-jazzy-joint-state-broadcaster ros-jazzy-ros2controlcli ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher ros-jazzy-control-msgs python3-numpy ffmpeg \
 && rm -rf /var/lib/apt/lists/*
# Menagerie UR5e: fetched at build time, verified against the checked-in sums.
COPY tools/models/menagerie.py models/universal_robots_ur5e/SHA256SUMS /tmp/models/
RUN : "${MENAGERIE_COMMIT:?set MENAGERIE_COMMIT}" \
 && python3 /tmp/models/menagerie.py fetch "$MENAGERIE_COMMIT" /opt/models/menagerie/universal_robots_ur5e \
 && python3 /tmp/models/menagerie.py verify /opt/models/menagerie/universal_robots_ur5e /tmp/models/SHA256SUMS \
 && rm -rf /tmp/models
ENV MENAGERIE_UR5E_DIR=/opt/models/menagerie/universal_robots_ur5e
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws

FROM dev AS build
COPY . /ws
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --base-paths src --build-base build/release --install-base install/release \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Runtime: the pinned ros-core image (no compilers, no colcon) plus the installed artifacts.
FROM ${RUNTIME_IMAGE} AS runtime
COPY --from=build /ws/install/release /opt/robotics
ENTRYPOINT ["/opt/robotics/runtime_core/lib/runtime_core/logic_sim"]
