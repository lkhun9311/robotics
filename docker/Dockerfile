# syntax=docker/dockerfile:1
# SPDX-License-Identifier: Apache-2.0
ARG ROS_IMAGE
FROM ${ROS_IMAGE} AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

FROM base AS dev
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libgtest-dev clang-format clang-tidy gdb valgrind gcovr \
      python3-pytest can-utils iproute2 rt-tests util-linux kmod \
 && rm -rf /var/lib/apt/lists/*
ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws
