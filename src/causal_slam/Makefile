SHELL := /bin/bash
.ONESHELL:
.SHELLFLAGS := -e -o pipefail -c
.DEFAULT_GOAL := help

PKG := causal_slam
PKG_DIR := $(CURDIR)
WS_DIR := $(abspath $(PKG_DIR)/../..)

ROS_SETUP := /opt/ros/jazzy/setup.bash
WS_SETUP := $(WS_DIR)/install/setup.bash

JOBS ?= $(shell nproc)
COLCON_WORKERS ?= $(JOBS)
BUILD_TYPE ?= RelWithDebInfo
BUILD_TESTING ?= ON

LOG_DIR := $(WS_DIR)/log
MANUAL_LOG_DIR := $(LOG_DIR)/manual
SMOKE_LOG_DIR := $(LOG_DIR)/smoke
BUILD_LOG := $(MANUAL_LOG_DIR)/$(PKG)_build.log

GATE_IMU_TOPIC ?= /imu/data
GATE_LIDAR_TOPIC ?= /points_raw
GATE_CHECKED_LIDAR_TOPIC ?= /points_checked
GATE_TF_TARGET_FRAME ?= odom
GATE_TF_SOURCE_FRAME ?= lidar
GATE_MODE ?= drop_degraded

WARN_RE := warning:|error:|fatal error|undefined reference|CMake Error|FAILED|not found:|Unable to find required file

COLCON_BUILD := \
	colcon build \
		--packages-select $(PKG) \
		--parallel-workers $(COLCON_WORKERS) \
		--event-handlers console_direct+ \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DBUILD_TESTING=$(BUILD_TESTING)

.PHONY: help dirs build clean-build rebuild test arch smoke ci warnings check-warnings gate demo-healthy demo-imu-lag demo-imu-drop demo-tf-missing clean-bak clean-generated clean-tmp clean-colcon-logs clean-nested clean-local clean-all status

help:
	@echo "Causal-SLAM Make targets"
	@echo
	@echo "Build:"
	@echo "  make build              build package"
	@echo "  make clean-build        remove package build/install and build"
	@echo "  make rebuild            alias for clean-build"
	@echo "  make test               run package tests"
	@echo "  make ci                 clean build + test + arch + smoke + warning check"
	@echo
	@echo "Checks:"
	@echo "  make arch               architecture dependency check"
	@echo "  make smoke              temporal gate smoke test"
	@echo "  make warnings           show warnings/errors from last build log"
	@echo "  make check-warnings     fail if build log contains warnings/errors"
	@echo "  make status             git status + workspace tree"
	@echo
	@echo "Runtime:"
	@echo "  make gate               launch integration temporal gate"
	@echo
	@echo "Runtime demo:"
	@echo "  make demo-healthy       launch healthy temporal gate demo"
	@echo "  make demo-imu-lag       launch IMU timestamp lag demo"
	@echo "  make demo-imu-drop      launch IMU dropped samples demo"
	@echo "  make demo-tf-missing    launch missing TF demo"
	@echo
	@echo "Clean:"
	@echo "  make clean-bak          remove *.bak_* files"
	@echo "  make clean-generated    remove generated helper input files"
	@echo "  make clean-tmp          remove /tmp/causal_slam* files"
	@echo "  make clean-colcon-logs  remove old colcon logs"
	@echo "  make clean-nested       remove accidental src/build src/install src/log"
	@echo "  make clean-local        clean clutter, keep build/install"
	@echo "  make clean-all          clean clutter + package build/install"
	@echo
	@echo "Variables:"
	@echo "  JOBS=$(JOBS)"
	@echo "  COLCON_WORKERS=$(COLCON_WORKERS)"
	@echo "  BUILD_TYPE=$(BUILD_TYPE)"
	@echo "  BUILD_TESTING=$(BUILD_TESTING)"
	@echo "  GATE_IMU_TOPIC=$(GATE_IMU_TOPIC)"
	@echo "  GATE_LIDAR_TOPIC=$(GATE_LIDAR_TOPIC)"
	@echo "  GATE_CHECKED_LIDAR_TOPIC=$(GATE_CHECKED_LIDAR_TOPIC)"
	@echo "  GATE_TF_TARGET_FRAME=$(GATE_TF_TARGET_FRAME)"
	@echo "  GATE_TF_SOURCE_FRAME=$(GATE_TF_SOURCE_FRAME)"
	@echo "  GATE_MODE=$(GATE_MODE)"
	@echo
	@echo "Examples:"
	@echo "  make ci"
	@echo "  make build JOBS=16"
	@echo "  make clean-build BUILD_TYPE=Debug"
	@echo "  make gate GATE_IMU_TOPIC=/livox/imu GATE_LIDAR_TOPIC=/livox/lidar GATE_CHECKED_LIDAR_TOPIC=/livox/lidar_checked"

dirs:
	mkdir -p "$(MANUAL_LOG_DIR)" "$(SMOKE_LOG_DIR)"

build: dirs
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	CMAKE_BUILD_PARALLEL_LEVEL="$(JOBS)" $(COLCON_BUILD) 2>&1 | tee "$(BUILD_LOG)"

clean-build: dirs
	cd "$(WS_DIR)"
	rm -rf "build/$(PKG)" "install/$(PKG)" "log/latest_build"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	CMAKE_BUILD_PARALLEL_LEVEL="$(JOBS)" $(COLCON_BUILD) 2>&1 | tee "$(BUILD_LOG)"

rebuild: clean-build

test:
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	test -f "$(WS_SETUP)"
	source "$(WS_SETUP)"
	colcon test --packages-select "$(PKG)" --parallel-workers "$(COLCON_WORKERS)" --event-handlers console_direct+
	colcon test-result --verbose

arch:
	cd "$(PKG_DIR)"
	./scripts/check_arch_deps.py

smoke: dirs
	cd "$(PKG_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	test -f "$(WS_SETUP)"
	source "$(WS_SETUP)"
	CAUSAL_SLAM_SMOKE_LOG_DIR="$(SMOKE_LOG_DIR)" \
	CAUSAL_SLAM_SMOKE_HTML_DIR="$(SMOKE_LOG_DIR)" \
	./scripts/smoke_temporal_gate.sh

ci: clean-build test arch smoke check-warnings

warnings:
	if [[ ! -f "$(BUILD_LOG)" ]]; then
		echo "No build log found: $(BUILD_LOG)"
		exit 0
	fi
	grep -nEi "$(WARN_RE)" "$(BUILD_LOG)" || true

check-warnings:
	if [[ ! -f "$(BUILD_LOG)" ]]; then
		echo "No build log found: $(BUILD_LOG)"
		exit 1
	fi
	if grep -nEi "$(WARN_RE)" "$(BUILD_LOG)"; then
		echo "Build log contains warnings/errors"
		exit 1
	fi
	echo "Build log warnings/errors: none"

gate: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate.launch.py \
		imu_topic:="$(GATE_IMU_TOPIC)" \
		lidar_topic:="$(GATE_LIDAR_TOPIC)" \
		checked_lidar_topic:="$(GATE_CHECKED_LIDAR_TOPIC)" \
		tf_target_frame:="$(GATE_TF_TARGET_FRAME)" \
		tf_source_frame:="$(GATE_TF_SOURCE_FRAME)" \
		lidar_gate_mode:="$(GATE_MODE)"

demo-healthy: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate_demo.launch.py fault_mode:=none tf_mode:=healthy

demo-imu-lag: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate_demo.launch.py fault_mode:=imu_lag tf_mode:=healthy

demo-imu-drop: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate_demo.launch.py fault_mode:=imu_drop tf_mode:=healthy

demo-tf-missing: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate_demo.launch.py fault_mode:=none tf_mode:=missing

clean-bak:
	cd "$(PKG_DIR)"
	find . -name '*.bak_*' -print -delete

clean-generated:
	cd "$(PKG_DIR)"
	rm -f scan_scoped_decision_inputs.txt
	rm -f fault_injection_inputs.txt
	rm -f collect_causal_slam_decision_inputs.sh

clean-tmp:
	find /tmp -maxdepth 1 -name 'causal_slam*' -print -delete 2>/dev/null || true

clean-colcon-logs:
	cd "$(WS_DIR)"
	rm -rf log/build_* log/test_* log/test-result_* log/latest log/latest_*

clean-nested:
	cd "$(WS_DIR)"
	rm -rf src/build src/install src/log
	rm -rf "$(PKG_DIR)/build" "$(PKG_DIR)/install" "$(PKG_DIR)/log"

clean-local: clean-bak clean-generated clean-tmp clean-nested

clean-all: clean-local clean-colcon-logs
	cd "$(WS_DIR)"
	rm -rf "build/$(PKG)" "install/$(PKG)"

status:
	cd "$(WS_DIR)"
	git -C "$(PKG_DIR)" status --short
	echo
	tree -L 2