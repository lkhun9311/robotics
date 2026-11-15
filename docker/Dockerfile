# syntax=docker/dockerfile:1
# SPDX-License-Identifier: Apache-2.0
ARG ROS_IMAGE
# Default keeps every stage's FROM resolvable when only the dev target is built (BuildKit
# resolves all stages); release builds pass the pinned digest from .env.
ARG RUNTIME_IMAGE=ros:jazzy-ros-core
# Base of the build stage: the dev stage below, or an already built dev image. CI passes the dev
# image it has just checked, so the runtime build neither rebuilds the dev stage nor fetches the
# model again when the layer cache misses, and ships from the very image the checks ran in.
ARG BUILD_FROM=dev
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
      python3-matplotlib \
      ros-jazzy-lely-core-libraries \
 && rm -rf /var/lib/apt/lists/*
# Menagerie UR5e: fetched at build time, verified against the checked-in sums.
COPY tools/models/menagerie.py models/universal_robots_ur5e/SHA256SUMS /tmp/models/
RUN : "${MENAGERIE_COMMIT:?set MENAGERIE_COMMIT}" \
 && python3 /tmp/models/menagerie.py fetch "$MENAGERIE_COMMIT" /opt/models/menagerie/universal_robots_ur5e \
 && python3 /tmp/models/menagerie.py verify /opt/models/menagerie/universal_robots_ur5e /tmp/models/SHA256SUMS \
 && rm -rf /tmp/models
ENV MENAGERIE_UR5E_DIR=/opt/models/menagerie/universal_robots_ur5e
# MuJoCo Python bindings for the simulation checks' forward kinematics and the replay render:
# the PyPI wheel of the same MuJoCo version as ros-jazzy-mujoco-vendor (3.12.0), pinned by its
# SHA-256. --no-deps: its runtime imports come from the Ubuntu archive (typing_extensions, and
# PyOpenGL for EGL rendering); glfw, etils and absl are needed only by viewers and extras.
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-typing-extensions python3-opengl \
 && rm -rf /var/lib/apt/lists/* \
 && printf '%s\n' 'mujoco==3.12.0 --hash=sha256:7ec16ce408871a0a9157cc556958ab66cd34db9fc1dccd3ef07717170163a4e0' \
      > /tmp/mujoco-requirements.txt \
 && pip install --no-cache-dir --no-deps --only-binary=:all: --require-hashes --break-system-packages \
      -r /tmp/mujoco-requirements.txt \
 && rm /tmp/mujoco-requirements.txt \
 && python3 -c 'import mujoco; v = mujoco.mj_versionString(); assert v == "3.12.0", v'
# CANopenNode (CANopenLinux canopend), the independent slave of the E3c interop tests. Both
# tarballs are GitHub archives of a commit (no git clone), pinned by SHA-256; the CANopenNode
# tree goes where the CANopenLinux Makefile expects its submodule. The build differs from the
# shipped one in docker/canopennode_build.h only (no self-start to Operational, log to stderr).
ARG CANOPENLINUX_COMMIT=87cfbac52bc67809be20ca03ef3533293e7f7c3a
ARG CANOPENLINUX_SHA256=309935638147a097967c61f35583d65ffe3947e76006b02f9921b0dd82563426
ARG CANOPENNODE_COMMIT=76b43c88ef6d5490cb2f1518e10646e8dcb45c76
ARG CANOPENNODE_SHA256=e124ae6402c259a796f4ae5fb539351f3c5d3904f8870eb9205773285dcc4358
COPY docker/canopennode_build.h /opt/canopennode/canopennode_build.h
RUN apt-get update \
 && apt-get install -y --no-install-recommends ca-certificates curl \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir /tmp/co && cd /tmp/co \
 && curl -fsSL -o cl.tgz "https://github.com/CANopenNode/CANopenLinux/archive/${CANOPENLINUX_COMMIT}.tar.gz" \
 && curl -fsSL -o cn.tgz "https://github.com/CANopenNode/CANopenNode/archive/${CANOPENNODE_COMMIT}.tar.gz" \
 && echo "${CANOPENLINUX_SHA256}  cl.tgz" | sha256sum -c - \
 && echo "${CANOPENNODE_SHA256}  cn.tgz" | sha256sum -c - \
 && mkdir -p linux/CANopenNode \
 && tar xzf cl.tgz -C linux --strip-components=1 \
 && tar xzf cn.tgz -C linux/CANopenNode --strip-components=1 \
 && make -C linux canopend OPT="-g -DCO_SINGLE_THREAD -include /opt/canopennode/canopennode_build.h" \
 && install -D linux/canopend /opt/canopennode/bin/canopend \
 && install -D linux/LICENSE /opt/canopennode/LICENSE.CANopenLinux \
 && install -D linux/CANopenNode/LICENSE /opt/canopennode/LICENSE.CANopenNode \
 && install -D linux/CANopenNode/example/OD.c /opt/canopennode/OD.c \
 && echo "${CANOPENLINUX_COMMIT} ${CANOPENNODE_COMMIT}" > /opt/canopennode/COMMITS \
 && cd / && rm -rf /tmp/co
ENV PATH=/opt/canopennode/bin:${PATH}
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws

FROM ${BUILD_FROM} AS build
COPY . /ws
# lely_peer is a test peer linked against the dev image's Lely libraries; it does not ship.
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --base-paths src --build-base build/release --install-base install/release \
      --packages-skip lely_peer --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Runtime: the pinned ros-core image (no compilers, no colcon) plus the installed artifacts.
FROM ${RUNTIME_IMAGE} AS runtime
COPY --from=build /ws/install/release /opt/robotics
# The bus processes ship too: each must load and start in this image (--help exits 0).
RUN /opt/robotics/vdrive_bank/lib/vdrive_bank/vdrive_bank --help \
 && /opt/robotics/bus_fault_proxy/lib/bus_fault_proxy/bus_fault_proxy --help
ENTRYPOINT ["/opt/robotics/runtime_core/lib/runtime_core/logic_sim"]
