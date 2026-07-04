SHELL := /bin/bash
.ONESHELL:
.SHELLFLAGS := -e -o pipefail -c
.DEFAULT_GOAL := help
.NOTPARALLEL:

PKG := causal_slam
PKG_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS_DIR := $(abspath $(PKG_DIR)/../..)

ROS_SETUP := /opt/ros/jazzy/setup.bash
WS_SETUP := $(WS_DIR)/install/setup.bash

BUILD_TYPE ?= RelWithDebInfo
BUILD_TESTING ?= ON
JOBS ?= $(shell nproc)
COLCON_WORKERS ?= $(JOBS)

LOG_DIR := $(WS_DIR)/log
MANUAL_LOG_DIR := $(LOG_DIR)/manual
SMOKE_LOG_DIR := $(LOG_DIR)/smoke
BUILD_LOG := $(MANUAL_LOG_DIR)/$(PKG)_build.log
CPPCHECK_LOG := $(MANUAL_LOG_DIR)/cppcheck.log

GATE_CONFIG ?= $(PKG_DIR)/config/temporal_gate.yaml

CLANG_FORMAT ?= clang-format
CPPCHECK ?= cppcheck

WARN_RE := warning:|error:|fatal error|undefined reference|CMake Error|FAILED|not found:|Unable to find required file

COLCON_BUILD := \
	colcon build \
		--base-paths "$(PKG_DIR)" \
		--packages-select "$(PKG)" \
		--parallel-workers "$(COLCON_WORKERS)" \
		--event-handlers console_direct+ \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
			-DBUILD_TESTING=$(BUILD_TESTING) \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

.PHONY: \
	help build test check ci format clean gate smoke runtime-stress status \
	_dirs _format-check _arch-check _cppcheck _warning-check

help:
	@echo "Causal-SLAM"
	@echo
	@echo "Main:"
	@echo "  make build           build ROS2 package"
	@echo "  make test            build and run tests"
	@echo "  make check           format-check + arch-check + cppcheck + build-log warnings"
	@echo "  make ci              clean + test + check + smoke"
	@echo "  make format          apply clang-format"
	@echo "  make clean           remove generated/build clutter"
	@echo
	@echo "Runtime:"
	@echo "  make gate            launch temporal gate with GATE_CONFIG"
	@echo "  make smoke           run smoke temporal gate scenario"
	@echo "  make runtime-stress  run heavy runtime stress scenario"
	@echo
	@echo "Debug:"
	@echo "  make status          git status and package tree"
	@echo
	@echo "Paths:"
	@echo "  PKG_DIR=$(PKG_DIR)"
	@echo "  WS_DIR=$(WS_DIR)"
	@echo
	@echo "Variables:"
	@echo "  BUILD_TYPE=$(BUILD_TYPE)"
	@echo "  BUILD_TESTING=$(BUILD_TESTING)"
	@echo "  JOBS=$(JOBS)"
	@echo "  COLCON_WORKERS=$(COLCON_WORKERS)"
	@echo "  GATE_CONFIG=$(GATE_CONFIG)"

_dirs:
	mkdir -p "$(MANUAL_LOG_DIR)" "$(SMOKE_LOG_DIR)"

build: _dirs
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	CMAKE_BUILD_PARALLEL_LEVEL="$(JOBS)" $(COLCON_BUILD) 2>&1 | tee "$(BUILD_LOG)"

test: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	colcon test \
		--packages-select "$(PKG)" \
		--parallel-workers "$(COLCON_WORKERS)" \
		--event-handlers console_direct+
	colcon test-result --verbose

check: _format-check _arch-check _cppcheck _warning-check

ci: clean test check smoke

format:
	cd "$(PKG_DIR)"
	if ! command -v "$(CLANG_FORMAT)" >/dev/null 2>&1; then
		echo "Missing $(CLANG_FORMAT). Install clang-format first."
		exit 1
	fi
	mapfile -d '' files < <(find src tests tools -type f \
		\( -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o \
		   -name '*.h' -o -name '*.hpp' -o -name '*.hh' \) \
		-print0)
	if (( $${#files[@]} == 0 )); then
		echo "No C++ files found for clang-format"
		exit 0
	fi
	"$(CLANG_FORMAT)" --style=file -i "$${files[@]}"
	echo "clang-format applied to $${#files[@]} files"

gate: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	ros2 launch "$(PKG)" temporal_gate_config.launch.py \
		config_file:="$(GATE_CONFIG)"

smoke: build
	cd "$(PKG_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	CAUSAL_SLAM_SMOKE_LOG_DIR="$(SMOKE_LOG_DIR)" \
	CAUSAL_SLAM_SMOKE_HTML_DIR="$(SMOKE_LOG_DIR)" \
	./scripts/smoke_temporal_gate.sh

runtime-stress: build
	cd "$(WS_DIR)"
	unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
	source "$(ROS_SETUP)"
	source "$(WS_SETUP)"
	CAUSAL_SLAM_RUNTIME_STRESS_GATE_MODE=observe \
	CAUSAL_SLAM_RUNTIME_STRESS_QOS_RELIABILITY=reliable \
	CAUSAL_SLAM_RUNTIME_STRESS_QOS_DEPTH=20 \
	CAUSAL_SLAM_RUNTIME_STRESS_POINT_COUNT=1000000 \
	CAUSAL_SLAM_RUNTIME_STRESS_LIDAR_PERIOD_MS=100.0 \
	CAUSAL_SLAM_RUNTIME_STRESS_DURATION_SEC=30 \
	"$(PKG_DIR)/scripts/runtime_stress_temporal_gate.sh"

clean:
	cd "$(WS_DIR)"
	rm -rf "build/$(PKG)" "install/$(PKG)"
	rm -rf src/build src/install src/log
	rm -rf "$(PKG_DIR)/build" "$(PKG_DIR)/install" "$(PKG_DIR)/log"
	rm -rf log/latest log/latest_build log/latest_test log/latest_test-result
	find "$(PKG_DIR)" -name '*.bak_*' -print -delete
	rm -f "$(PKG_DIR)/scan_scoped_decision_inputs.txt"
	rm -f "$(PKG_DIR)/fault_injection_inputs.txt"
	rm -f "$(PKG_DIR)/collect_causal_slam_decision_inputs.sh"
	find /tmp -maxdepth 1 -name 'causal_slam*' -print -delete 2>/dev/null || true

status:
	cd "$(PKG_DIR)"
	git status --short
	echo
	tree -L 2

_format-check:
	cd "$(PKG_DIR)"
	if ! command -v "$(CLANG_FORMAT)" >/dev/null 2>&1; then
		echo "Missing $(CLANG_FORMAT). Install clang-format first."
		exit 1
	fi
	mapfile -d '' files < <(find src tests tools -type f \
		\( -name '*.cc' -o -name '*.cpp' -o -name '*.cxx' -o \
		   -name '*.h' -o -name '*.hpp' -o -name '*.hh' \) \
		-print0)
	if (( $${#files[@]} == 0 )); then
		echo "No C++ files found for clang-format"
		exit 0
	fi
	"$(CLANG_FORMAT)" --style=file --dry-run --Werror "$${files[@]}"
	echo "clang-format check: OK"

_arch-check:
	cd "$(PKG_DIR)"
	./scripts/check_arch_deps.py

_cppcheck: build _dirs
	cd "$(WS_DIR)"
	if ! command -v "$(CPPCHECK)" >/dev/null 2>&1; then
		echo "Missing $(CPPCHECK). Install cppcheck first."
		exit 1
	fi
	test -f "$(WS_DIR)/build/$(PKG)/compile_commands.json"
	"$(CPPCHECK)" \
		--project="$(WS_DIR)/build/$(PKG)/compile_commands.json" \
		--file-filter="$(PKG_DIR)/src/*" \
		--file-filter="$(PKG_DIR)/tools/*" \
		--std=c++20 \
		--enable=warning,performance,portability \
		--inline-suppr \
		--suppress=missingIncludeSystem \
		--suppress=passedByValueCallback \
		--suppress=useStlAlgorithm \
		--suppress=useInitializationList \
		--template=gcc \
		--error-exitcode=1 \
		2>&1 | tee "$(CPPCHECK_LOG)"
	echo "cppcheck: OK"

_warning-check:
	if [[ ! -f "$(BUILD_LOG)" ]]; then
		echo "No build log found: $(BUILD_LOG)"
		exit 1
	fi
	if grep -nEi "$(WARN_RE)" "$(BUILD_LOG)"; then
		echo "Build log contains warnings/errors"
		exit 1
	fi
	echo "Build log warnings/errors: none"