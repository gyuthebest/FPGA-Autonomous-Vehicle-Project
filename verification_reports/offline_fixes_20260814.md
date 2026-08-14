# 보드 없이 수행한 수정 및 검증 (2026-08-14)

FPGA 보드를 연결할 수 없는 상태에서 수정 가능한 항목만 처리했다.
**RTL(sources_1/new/*.sv, packaged IP)은 한 줄도 변경하지 않았다.** 변경은
전부 CARLA 측 Python이며, 기존 위험도/신뢰도 알고리즘과 파라미터는 그대로다.

## 1. 결론 요약

| 항목 | 결과 |
|---|---|
| PL self-checking 회귀 (`tb_pl_full_verification`) | **88 PASS / 0 FAIL** |
| Risk/Reliability gap 회귀 (`tb_risk_reliability_matrix`) | **115 PASS / 0 FAIL** |
| CARLA AXI 재생 smoke (`tb_carla_axi_replay`) | **5 samples / 0 FAIL** |
| Python 단위 시험 (`test_scenario_pl_alignment`) | **17 PASS** (기존 7 + 신규 10) |
| 잡음/패킹 검증 (`verify_sensor_noise`) | **9개 항목 PASS** |

RTL 회귀 합계 **203 PASS / 0 FAIL**로, codex 최종 기록과 동일하다. RTL이
변경되지 않았음을 회귀 결과로도 확인한 것이다.

## 2. 정상 주행에서 DEGRADED가 뜨던 원인

가설("잡음이 없어서")은 **일부만** 맞다. 채널별로 원인이 다르다.

### 2.1 잡음 부재가 원인인 것 (이번에 해결)

`sensor_checker.sv`의 stuck 판정은 다음과 같다.

```systemverilog
if (processed_data != 0) raw_stuck = 2'b00;   // stuck_cnt -= STUCK_D
else if (testable)       raw_stuck = 2'b01;   // stuck_cnt += STUCK_U
```

온도/습도/조도는 `CHANNEL_TYPE_2 = 0`이라 `cond_b = 1'b1`, 즉 **항상
testable**이다. `STUCK_N/STUCK_U = 15`이므로 값이 15표본(20 Hz에서 0.75초)
동안 변하지 않으면 무조건 stuck이 확정된다.

측정한 원인:

- CARLA IMU 블루프린트의 `noise_accel_stddev_*`, `noise_gyro_stddev_*`가 전부 `0.0`
- 온도/습도는 `weather_manager`가 상수로 주입
- 조도는 `SUN_UPDATE_INTERVAL_FRAMES = 30`이라 30프레임마다 한 번만 갱신
- 일정 반경 선회 중에는 요각속도가 상수라 `delta_gyro_z == 0`이 이어짐

`verify_sensor_noise.py`의 [1]번 시험이 잡음 없이 이 오탐을 재현한다.
400표본(20초) 기준:

| 시나리오 | stuck 발생 채널 (프레임 수) |
|---|---|
| 정지 | temperature(386), humidity(386), lux(360) |
| 직선 정속 | temperature(386), humidity(386), lux(360) |
| 일정 반경 선회 | **gyro_z(390)**, temperature(386), humidity(386), lux(360) |
| 가속 | temperature(386), humidity(386), lux(360) |

codex 보고서(`pl_validation_continuation_20260813.md` §4.3)가 실측한
temperature/humidity/lux 1,030프레임, gyro_z 101프레임과 같은 현상이다.

### 2.2 잡음으로 해결되지 **않는** 것 (미해결로 남김)

- **Jump/Noise 오탐** (distance 1,010프레임, approach_speed 987프레임):
  임계값 자체가 정상 레이더 변동보다 작다. 잡음을 더하면 **악화**된다.
- **Consistency 오탐** (accel_x 1,034, gyro_z 858): `pred_data_t`의 signed
  16비트 wrap 문제다. 잡음과 무관하다.

이 둘은 파라미터/Q-format 결정이 필요하므로 이번에 건드리지 않았다.

## 3. 적용한 수정

### 3.1 기본 센서 측정 잡음 — `sensor_noise.py` (신규)

**백색 잡음을 쓰지 않았다.** noise 검사기가 두 조건의 OR이기 때문이다.

```systemverilog
noise_error = (delta_sum > NOISE_THRESHOLD_1 * HISTORY)
           || ($countones(flip_history) > NOISE_THRESHOLD_2);   // NOISE_THRESHOLD_2 = 7
```

백색 잡음의 1차 차분은 연속 표본끼리 강한 음의 상관을 가져 부호가 거의 매
표본 뒤집힌다. 기대 반전 횟수가 약 6.7/10으로 임계값 7에 붙어버려 간헐적
noise 오탐을 만든다. 그래서 **대역 제한된 저주파 정현파 합**을 사용했다.
진폭이 유한하게 묶여 range/jump/분류 임계값을 넘지 않고, 부호가 여러 표본
유지되며, 표본당 |delta|는 1 LSB 이상이다.

측정된 여유 (정지 시나리오, 10표본 창):

| 채널 | delta 합 / 한계 | 여유 | 부호반전 / 한계 |
|---|---|---|---|
| temperature | 6 / 20 | 3.3x | 3 / 7 |
| humidity | 6 / 20 | 3.3x | 3 / 7 |
| lux | 955 / 50000 | 52.4x | 2 / 7 |
| gyro_z | 22 / 2500 | 113.6x | 2 / 7 |
| accel_z | 19 / 5000 | 263.2x | 2 / 7 |

`CARLA_SENSOR_NOISE=0`으로 끌 수 있다. 표본 카운터 기반이라 결정론적이며,
같은 표본 수를 진행하면 항상 같은 파형이 나온다(캡처 재생 비교용).

**distance / approach_speed에는 잡음을 넣지 않았다.** `sensor_reliability.sv`가
무표적 상태를 `distance == 15'd20000 && approach_speed == 13'sd0`으로 정확히
판정하기 때문에, 1 LSB만 흔들려도 sentinel이 깨져 distance 진단 오탐이
되살아난다. 두 채널은 레이더 측정값 자체가 이미 프레임마다 변한다.

### 3.2 호출 순서

```text
물리 시나리오 -> 고장 주입 -> 측정 잡음(고장 채널 skip)
```

잡음을 고장 주입 **뒤**에 두고 `skip=injector.frozen_channels`를 넘긴다.

- 앞에 두면: 위험도 주입(`_apply_risk_faults`)이 온도 `-60.0`, 습도 `95.0`을
  상수로 덮어써서 노면 위험도 시험이 매번 온습도 stuck까지 만들어낸다.
  즉 위험도 시험이 신뢰도 시험을 오염시킨다.
- 뒤에 두되 skip이 없으면: stuck 고장 주입이 상수를 쓰는 방식이라 잡음이
  주입한 고장을 지워버린다.

두 경우를 `verify_sensor_noise.py` [7]번 시험이 모두 검사한다.

### 3.3 distance range 고장 도달성 — 사용자 지적이 맞았다

`control_panel.py`에는 다음 주석이 있었다.

> Distance is unsigned and its AXI maximum (200.00 m) is also the configured
> range ceiling. No out-of-range bit pattern can reach the PL for this channel.

**이 판단은 틀렸다.** distance 필드는 15비트 부호 없음이라 0..32767
(0..327.67 m)을 표현하고, PL의 `RANGE_THRESHOLD_MAX`는 20000이다. 즉
20001..32767은 표현 가능하면서 PL이 range fault로 판정하는 구간이다.
실제로 막고 있던 것은 PL이 아니라 `build_input_words`의 자기 포화였다.

```python
distance_q = _quantize_unsigned(min(float(distance_m), 200.0), 100.0, 15)   # 이전
distance_q = _quantize_unsigned(float(distance_m), 100.0, 15)               # 이후
```

레이더 실측은 `main.py`에서 이미 200 m로 제한되므로 정상 주행 동작은 변하지
않고, sentinel(정확히 20000)도 유지된다. `RANGE_SENSORS`에 distance를
추가하고 주입값을 250.0 m(= 25000 LSB)로 정의했다.

검증 [5]: 250 m -> 25000 -> range fault 확정, sentinel 20000은 오판 없음.

### 3.4 앞차가 도로를 가로질러 서 있던 문제

원인은 스폰 waypoint 선택이다. 네 곳 모두 다음 형태였다.

```python
target_wp = min(next_waypoints, key=lambda wp: heading_error(wp))
```

`min()`은 후보가 **전부** 교차로 분기여도 "가장 덜 어긋난" 것을 무조건
반환한다. Town04 인터체인지/분기점에서 진행 방향과 90도 어긋난 waypoint가
선택되어 차량이 도로를 가로질러 스폰됐다.

`utils.select_aligned_waypoint()`를 추가해 **허용 각도(기본 25도)를 넘거나
교차로 내부인 waypoint를 후보에서 제외**하고, 남는 후보가 없으면 `None`을
돌려 호출부가 이번 스폰을 건너뛰게 했다. 적용 위치:

- `world_scenario_controller._spawn_safe_obstacle` (제어판 충돌 시나리오)
- `obstacle_manager.update_highway` / `update_mountain` (차량)
- `obstacle_manager.update_school` (보행자)
- `obstacle_manager.update_city` (방지턱, 기존 `is_intersection` 확인 대체)

같은 함수에서 보행자 오프셋 버그도 고쳤다. `transform.location.y += 2.5`는
**월드 좌표 Y축**에 더하는 코드라 도로가 동서 방향일 때 보행자가 차선
한가운데나 반대 차선에 나타났다. 차선 기준 우측 벡터로 오프셋하도록 바꿨다.

단위 시험 6건 신규 추가(정렬 선택, 90도 거부, 교차로 거부, 359/1도 wraparound,
역주행 180도 거부, 빈 후보).

### 3.5 1인칭 시점 중앙 정렬

`camera_manager.py`의 1인칭 카메라가 `y=-0.4`(좌측 운전석 오프셋)였다.
`y=0.0`으로 차량 세로축 중앙에 맞췄다.

## 4. 변경 파일

신규: `sensor_noise.py`, `verify_sensor_noise.py`
수정: `main.py`, `control_panel.py`, `fpga_interface.py`, `camera_manager.py`,
`obstacle_manager.py`, `world_scenario_controller.py`, `utils.py`,
`test_scenario_pl_alignment.py`

RTL 및 packaged IP: **변경 없음**

## 5. 보드가 있어야 확인 가능한 잔여 항목

- 실제 CARLA 실행 중 1인칭 시점/장애물 배치 육안 확인
- 잡음이 들어간 실제 20 Hz 캡처에서 신뢰도 NORMAL 유지 확인
- distance range 버튼의 실제 PL 응답 확인
- 74개 라이브 시나리오 재확인
