# Causal-SLAM

Causal-SLAM — ROS 2 / C++ слой временной проверки для LiDAR/IMU SLAM/LIO pipeline.

Идея простая: **перед тем как доверять геометрии SLAM, сначала проверить, имели ли сенсорные данные право быть объединены во времени.**

Это не новый SLAM, не FAST-LIO/LIO-SAM replacement и не GUI. Это temporal monitor / guard перед SLAM.

## Что делает сейчас

Causal-SLAM проверяет:

- стабильность LiDAR/IMU потоков;
- покрытие LiDAR scan window данными IMU;
- наличие и пригодность time-полей в `PointCloud2`;
- TF lookup / stale / future transforms;
- можно ли делать map update.

Основные выходы:

```text
/causal_slam/map_update_allowed
/causal_slam/temporal_health
/causal_slam/map_update_reason
/causal_slam/fault_reasons
/causal_slam/map_update_decision_json
/causal_slam/checked_lidar
```

## Что не делает сейчас

Causal-SLAM пока не:

- исправляет timestamps;
- делает deskew correction;
- строит карту;
- заменяет SLAM estimator;
- гарантирует визуальное улучшение без подключения downstream SLAM к `/causal_slam/checked_lidar`.

## Сборка

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/causal_slam_ws/src
cd ~/causal_slam_ws/src
git clone <repo-url> causal_slam

cd ~/causal_slam_ws
colcon build --packages-select causal_slam
source install/setup.bash
```

Или из репозитория:

```bash
make build
```

## Online запуск

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file src/causal_slam/config/temporal_gate.yaml
```

Если запуск из корня репозитория:

```bash
ros2 run causal_slam temporal_monitor_node --ros-args \
  --params-file config/temporal_gate.yaml
```

Главный конфиг:

```text
config/temporal_gate.yaml
```

В нём настраиваются input topics, output topics, gate mode, IMU thresholds, LiDAR scan window и TF monitoring.

## Gate modes

```text
observe        только диагностика, без блокировки
drop_invalid   блокировать invalid состояния
drop_degraded  блокировать degraded и invalid
strict         пропускать только полностью healthy данные
```

Для подключения к SLAM downstream pipeline должен читать не raw LiDAR topic, а:

```text
/causal_slam/checked_lidar
```

## Offline rosbag analysis

Offline режим нужен как воспроизводимый стенд на датасетах.

```bash
ros2 run causal_slam causal_slam_analyze_bag \
  --bag /path/to/rosbag2 \
  --lidar-topic /ouster/points \
  --imu-topic /ouster/imu \
  --report /tmp/causal_slam_report.json \
  --html-report /tmp/causal_slam_report.html
```

Что ожидать:

- краткий console summary;
- JSON report;
- optional HTML report.

Главные поля JSON:

```text
verdict.health
verdict.reason
point_cloud2_capability
lidar_scan_windows
imu_coverage
stream_timing_faults
```

## Как читать результат

Health:

```text
OK        временная проверка прошла
WARNING   есть подозрительные, но не критичные признаки
DEGRADED  данные опасны для map update
INVALID   данных недостаточно или состояние некорректно
```

Map update:

```text
true   можно использовать данные для map update
false  temporal evidence degraded/invalid
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

## Планируемая визуальная проверка

Нужен датасет, где temporal faults реально портят SLAM/LIO результат.

План проверки:

```text
1. Запустить SLAM на raw LiDAR/IMU.
2. Запустить тот же SLAM через Causal-SLAM.
3. Подать в SLAM /causal_slam/checked_lidar вместо raw LiDAR.
4. Сравнить карту/траекторию до и после.
5. Показать side-by-side GIF.
```

Ожидаемый полезный случай:

```text
raw SLAM:
  карта мажется, двоится или получает плохие map updates

SLAM через Causal-SLAM:
  degraded/invalid temporal windows не попадают в map update
  карта визуально стабильнее
```

## Статус

Experimental MVP.

Сейчас проект полезен как temporal diagnostics/gating layer и offline evidence harness. Production-ready safety system из него ещё не сделан.
