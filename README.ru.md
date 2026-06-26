# Causal-SLAM

Causal-SLAM — небольшой ROS 2 / C++ инструмент для проверки временной целостности LiDAR/IMU SLAM-пайплайнов.

Идея проекта:

> Перед тем как доверять геометрии SLAM, сначала проверь корректность времени сенсорных данных.

Это не новый SLAM-алгоритм.  
Это temporal monitor / guard, который проверяет тайминги LiDAR, IMU и TF и сообщает, можно ли безопасно делать map update.

## Поддерживаемая версия

Проверено на:

- Ubuntu 24.04
- ROS 2 Jazzy
- C++20
- CMake 3.28+

ROS 1 поддерживается только как экспериментальный compatibility adapter.

## Что проверяется

- тайминг LiDAR-потока
- тайминг IMU-потока
- покрытие LiDAR scan window данными IMU
- задержанные или нестабильные timestamps
- диагностика time-полей в PointCloud2
- ошибки TF lookup
- устаревшие или будущие TF transforms
- решение allow/block для map update

## Сборка

    source /opt/ros/jazzy/setup.bash

    mkdir -p ~/causal_slam_ws/src
    cd ~/causal_slam_ws/src
    git clone <repo-url> causal_slam

    cd ~/causal_slam_ws
    colcon build --packages-select causal_slam
    source install/setup.bash

Или через Makefile:

    make build

## Тесты

    make test

Smoke-сценарии:

    make smoke

Smoke-тесты запускают небольшие LiDAR/IMU сценарии и проверяют healthy/faulty timing cases.

## Запуск

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    ros2 run causal_slam temporal_monitor_node --ros-args \
      --params-file src/causal_slam/config/temporal_gate.yaml

Если запускать из корня репозитория:

    ros2 run causal_slam temporal_monitor_node --ros-args \
      --params-file config/temporal_gate.yaml

## Конфигурация

Основной конфиг:

    config/temporal_gate.yaml

В нём настраиваются:

- input/output topics
- gate mode
- IMU thresholds
- LiDAR scan settings
- TF monitoring
- HTML report path

## Основные output topics

    /causal_slam/map_update_allowed
    /causal_slam/temporal_health
    /causal_slam/map_update_reason
    /causal_slam/fault_reasons
    /causal_slam/map_update_decision_json
    /causal_slam/checked_lidar

## Gate modes

    observe
    drop_invalid
    drop_degraded
    strict

Значение:

- observe: только диагностика, без блокировки LiDAR
- drop_invalid: блокировать только invalid temporal states
- drop_degraded: блокировать degraded и invalid states
- strict: пропускать только полностью healthy data

## Значения output

Health:

    OK
    WARNING
    DEGRADED
    INVALID

Map update:

    true  -> временные данные выглядят приемлемо
    false -> временные данные degraded или invalid

Примеры fault reasons:

    stream_timing_unstable
    imu_window_incomplete
    lidar_point_time_unsupported
    lidar_point_time_extraction_failed
    lidar_scan_window_low_confidence
    tf_lookup_failed
    tf_extrapolation_required
    tf_age_too_high
    tf_transform_from_future

## Статус

Experimental MVP.

Проект полезен для проверки временной целостности вокруг SLAM/LIO pipeline, но пока не является production-ready safety system.

## License

MIT