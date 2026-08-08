# CARLA 자율주행 FPGA 센서 명세서 (최종 정직본 - Sensor Fusion 적용)

본 문서는 CARLA 시뮬레이터에서 추출되어 FPGA로 인가되는 모든 센서 데이터들의 추출 방식 및 스펙을 정의합니다. **(시뮬레이터 참값을 우회하여 쓰지 않고, 실제 차량의 방식과 동일하게 IMU 센서 데이터를 파이썬 백엔드에서 자체 퓨전하여 전달하는 정직한 스펙입니다.)**

---

## 1. 센서 데이터 (Sensor Data) - `sensor_data_t`

| 변수명 | CARLA 실제 센서 출처 | 추출 및 가공 방식 (정직본) | 스케일 | 비트폭 (부호) | 범위 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **distance** | `sensor.other.radar` | 레이더 포인트 클라우드 중 최소 거리(Depth) 추출 | **x100** ($cm$) | `15 unsigned` | $0 \sim 20000$ (200m) |
| **approach_speed** | `sensor.other.radar` | 레이더에서 측정된 상대 속도 (Radial Velocity) | **x100** ($cm/s$) | `13 signed` | $-4000 \sim 4000$ |
| **speed_x, y, z** | `carla.Vehicle.get_velocity()` | 차량 내부 CAN 통신 휠 스피드 및 속도계 데이터 모사 | **x100** ($cm/s$) | `14 signed` | $-5555 \sim 5555$ |
| **accel_x, y, z** | `sensor.other.imu` | IMU 가속도계에서 측정된 3축 가속도 (중력 포함) | **x100** ($cm/s^2$) | `12 signed` | $-1600 \sim 1600$ |
| **gyro_x, y, z** | `sensor.other.imu` | IMU 자이로스코프에서 측정된 3축 각속도 | **x1000** ($mrad/s$) | `16 signed` | $-16000 \sim 16000$ |
| **incline_x, y** | `sensor.other.imu` (Sensor Fusion) | **[상보 필터 적용]** 가속도계가 측정한 중력 벡터(정적 기울기)와 자이로스코프의 각속도 적분값(동적 변화)을 98:2 비율로 융합하여 추정한 실제 차량과 완벽히 동일한 방식의 Pitch, Roll 추정값. | **x100** | `16 signed` | $-18000 \sim 18000$ |
| **incline_z** | `sensor.other.imu` (Compass) | IMU에 내장된 지자기 센서(Compass)로 측정한 절대 방위각 (Yaw) | **x100** | `16 signed` | $-18000 \sim 18000$ |
| **lux** | (Virtual Sensor) | 태양 고도 및 날씨(구름/안개) 변수를 통해 파이썬 자체 조도 연산 | **x1** | `18 unsigned` | $0 \sim 130000$ |

---

> **📝 개발자 노트 (기울기값 처리 관련)**
> - 기존 버전에서는 제어 알고리즘의 편의를 위해 시뮬레이터 물리 엔진의 3D 객체 자세(Ground Truth)를 직접 빼와 사용했습니다.
> - **현재 최종 버전에서는 참값 사용을 전면 배제**하고, 실제 자율주행 차량이 기울기를 얻어내는 원리인 **"IMU 가속도계 + 자이로스코프 상보 필터(Complementary Filter) 결합 및 나침반 센서 활용"**을 파이썬 스크립트 단에서 정직하게 구현하여 FPGA로 넘겨주도록 변경되었습니다.
> - 이에 따라 급가속이나 방지턱 충격 시, 실제 차량과 마찬가지로 미세한 Jitter나 오차가 발생할 수 있으며, FPGA는 이 노이즈를 견뎌내며 제어해야 합니다.
