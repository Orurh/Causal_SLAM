# Causal-SLAM

[English version](README.md)

**Causal-SLAM** — экспериментальный слой временной целостности для LiDAR/IMU SLAM/LIO-пайплайнов на ROS 2 и C++17.

> Перед тем как доверять геометрии, проверить, были ли измерения совместимы во времени.

Проект не строит карту и не заменяет FAST-LIO, LIO-SAM или другой SLAM/LIO estimator. Он анализирует входные данные до их использования и формирует диагностику, health-состояние и решение о допустимости дальнейшей обработки.

## Зачем это нужно

LiDAR-скан формируется в течение некоторого временного интервала, а IMU измеряет движение сенсора внутри этого интервала. Наличие обоих топиков и монотонных timestamp ещё не гарантирует, что конкретный скан корректно покрыт IMU-данными.

Пропуски, перестановка сообщений, неверная интерпретация времени точек или недоступный TF могут выглядеть дальше по пайплайну как проблемы регистрации, дрейф или нестабильное обновление карты.

Causal-SLAM задаёт более ранний вопрос:

**имели ли эти измерения право быть объединены во времени?**

## Архитектура

```text
LiDAR / IMU / TF / rosbag2
            |
            v
     Causal-SLAM
  temporal integrity layer
            |
            +--> diagnostics / health / evidence
            +--> map update decision
            +--> checked LiDAR
            |
            v
      SLAM / LIO estimator
```

Domain/application-ядро не зависит от `rclcpp`. ROS 2-слой отвечает за сообщения, QoS, TF, параметры, rosbag2 и адаптацию данных.

## Что реализовано

Causal-SLAM проверяет:

- стабильность LiDAR- и IMU-потоков;
- gaps, jitter и перестановку сообщений;
- временное окно LiDAR-скана;
- наличие и интерпретацию time-полей в `PointCloud2`;
- Ouster-style поля `t` и `offset_time` типа `UINT32`;
- покрытие LiDAR-окна данными IMU;
- пропуски на границах окна и внутренние разрывы;
- доступность, возраст и временную корректность TF;
- допустимость передачи LiDAR дальше и обновления карты.

Доступны:

- live ROS 2 monitor;
- checked LiDAR topic;
- structured diagnostics;
- JSON decision topic;
- offline-анализ rosbag2;
- console summary;
- JSON report;
- optional HTML rendering.

## Ограничения

На текущем этапе Causal-SLAM не:

- исправляет timestamp;
- выполняет deskew;
- оценивает калибровку или IMU bias;
- строит карту и траекторию;
- заменяет SLAM/LIO estimator;
- гарантирует улучшение карты на любых данных.

Проект должен сначала обнаруживать и объяснять временные нарушения. Контролируемые fault-injection эксперименты и сравнение downstream SLAM до/после проверки — следующий этап валидации.

## Требования

- Ubuntu с ROS 2 Jazzy;
- C++17;
- CMake 3.20+;
- `colcon`;
- rosbag2 для offline-анализа.

Runtime сейчас реализован для ROS 2. Совместимое с C++17 ядро оставляет возможность добавить отдельный ROS 1 Noetic adapter позже, но ROS 1 runtime пока не заявлен как готовый.

## Сборка

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/causal_slam_ws/src
cd ~/causal_slam_ws/src

git clone https://github.com/Orurh/Causal_SLAM.git causal_slam

cd ~/causal_slam_ws
colcon build \
  --packages-select causal_slam \
  --symlink-install

source install/setup.bash
```

Из корня репозитория также доступны команды Makefile:

```bash
make build
make test
make check
```

## Online monitor

```bash
source /opt/ros/jazzy/setup.bash
source ~/causal_slam_ws/install/setup.bash

ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file ~/causal_slam_ws/src/causal_slam/config/temporal_gate.yaml
```

Основной конфиг:

```text
config/temporal_gate.yaml
```

В нём задаются входные и выходные топики, QoS, gate mode, пороги IMU coverage, параметры LiDAR scan window и TF monitoring.

## Основные выходы

```text
/causal_slam/map_update_allowed
/causal_slam/temporal_health
/causal_slam/map_update_reason
/causal_slam/fault_reasons
/causal_slam/map_update_decision_json
/causal_slam/checked_lidar
```

Для реального gating downstream SLAM должен читать:

```text
/causal_slam/checked_lidar
```

вместо исходного LiDAR-топика.

## Gate modes

```text
observe
  только диагностика; LiDAR передаётся дальше

drop_invalid
  блокируются INVALID-состояния

drop_degraded
  блокируются INVALID-состояния и DEGRADED-состояния
  с hard fusion blocker; диагностический DEGRADED без такого
  блокера может быть передан дальше

strict
  передаются только полностью healthy данные после накопления
  минимально необходимого временного evidence
```

## Offline rosbag2 analysis

Offline-режим нужен для воспроизводимой проверки датасетов без артефактов live playback.

```bash
ros2 run causal_slam causal_slam_analyze_bag \
  --bag /path/to/rosbag2 \
  --lidar-topic /ouster/points \
  --imu-topic /ouster/imu \
  --report /tmp/causal_slam_report.json \
  --html-report /tmp/causal_slam_report.html
```

JSON — основной машинно-читаемый артефакт. HTML является дополнительным представлением отчёта.

Главные разделы JSON:

```text
verdict
point_cloud2_capability
lidar_scan_windows
imu_coverage
stream_timing_faults
```

## Интерпретация результата

```text
OK
  временная проверка пройдена

WARNING
  обнаружены подозрительные, но не критичные признаки

DEGRADED
  есть нарушения или недостаточная уверенность;
  решение зависит от причины и gate policy

INVALID
  данных недостаточно или состояние некорректно
```

Примеры fault reasons:

```text
lidar_stream_timing_jitter_high
lidar_stream_timing_short_period
lidar_stream_timing_long_period
imu_stream_timing_jitter_high
imu_stream_timing_jitter_suspicious
imu_window_incomplete
lidar_point_time_unsupported
lidar_point_time_extraction_failed
lidar_scan_window_low_confidence
tf_lookup_failed
tf_extrapolation_required
tf_age_too_high
tf_transform_from_future
```

## Текущая валидация

Проект проверяется на открытых LiDAR/IMU-датасетах. На корректных данных он должен подтверждать временную совместимость, а не искать неисправность любой ценой. Для неполных или неоднозначных данных отчёт должен показывать причину и уровень уверенности.

Ближайший этап:

1. контролируемо удалять IMU-сообщения;
2. добавлять временной offset;
3. менять порядок сообщений;
4. сравнивать диагностику и поведение downstream SLAM;
5. отделять временные причины от проблем геометрии, калибровки и estimator.

## Статус

**Experimental MVP.**

Сейчас проект полезен как temporal diagnostics/gating layer и воспроизводимый offline evidence harness. Это не production safety system.

Коротко об идее проекта:  
[«Имели ли LiDAR и IMU право быть объединены во времени?» — Сетка](https://setka.ru/posts/019f770c-7f4c-71b3-a53f-e7ac6af67b33)