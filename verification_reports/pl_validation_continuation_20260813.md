# PL 전체 검증 계속 수행 결과

## 1. 실행한 검증

- CARLA 정상 주행 캡처 `pl_capture_20260813_013323.csv`의 1,045개 프레임을
  AXI4-Lite를 통해 `top_controller`에 전부 재생했다.
- REG0~REG8을 먼저 적재하고 REG9(sample_seq)를 마지막에 쓰는 실제 PS 동작을
  그대로 사용했다.
- 각 프레임의 risk/reliability sequence, output valid, X/Z 발생 여부를 검사했다.
- Range, Jump, Stuck, Timeout, Noise, Consistency, 위험도 제어, TD/MRM 및 AXI 통합을
  대상으로 한 고장 주입 테스트벤치를 다시 실행했다.
- CARLA 입력 캡처와 RTL 출력을 sample_seq 기준으로 결합하는 독립 비교기를 추가했다.

## 2. 통과한 항목

| 항목 | 결과 |
|---|---:|
| CARLA 벡터 재생 | 1,045 / 1,045 성공 |
| AXI/파이프라인 sequence 불일치 | 0건 |
| output valid 오류 | 0건 |
| 출력 X/Z | 0건 |
| 고장 주입 자기검사 | PASS 75 / FAIL 0 |
| 외부 안전 불변조건 위반 | 0건 |

외부 안전 불변조건에는 다음을 포함했다.

- MRM일 때 accelerator=0, brake=3, hazard=1
- TD 플래그와 `td_remain_sec` 표현의 정합성
- PL이 입력 제한속도보다 높은 제한속도를 생성하지 않음
- risk/reliability 결과의 sample_seq가 입력 sample_seq와 일치

## 3. 정상 주행 캡처에서 발견한 문제

캡처의 모든 1,045개 샘플은 `fault_label=none`이다. 그러나 RTL 출력은 다음과 같다.

| 관찰 항목 | 프레임 수 |
|---|---:|
| 하나 이상의 INVALID 채널 | 101 |
| HUD warning | 101 |
| Transition Demand | 306 |
| MRM | 116 |
| accelerator 명령 변경 | 716 |
| brake 명령 변경 | 126 |

따라서 AXI 인터페이스와 파이프라인은 정상이나, 현재 신뢰도 검사 파라미터/기준식은
CARLA 정상 데이터에 대해 오탐을 만든다.

## 4. 검사기별 오탐 원인

### 4.1 Jump

| 채널 | 활성 프레임 | 최장 연속 |
|---|---:|---:|
| distance | 1,010 | 987 |
| approach_speed | 987 | 982 |

- 현재 distance Jump threshold는 PL 단위 100, approach_speed는 10이다.
- 정상 주행 캡처에서 distance 2차 차분 최대값은 534, approach_speed는 4,080이다.
- 특히 approach_speed는 0.08 m/s 단위로 압축되어 들어오는데 threshold=10은
  0.10 m/s 상당으로, 정상 레이더 변동에도 매우 민감하다.
- 첫 접근속도 원시값은 비현실적으로 커서 Python 패커에서 포화되고, 이후 정상값으로
  전환된다. 이 시작 과도상태도 Jump 증거를 만든다.

### 4.2 Noise

| 채널 | 활성 프레임 |
|---|---:|
| distance | 902 |
| approach_speed | 1,039 |
| accel_x | 151 |
| accel_y | 382 |
| gyro_z | 142 |

- 현재 Noise는 10샘플 평균 절대 delta 또는 부호 반전 횟수를 사용한다.
- 레이더의 정상 프레임 변동이 평균 delta 임계값을 계속 넘는다.
- CARLA 로그에서 정상 구간 분산을 계산해 threshold를 정하지 않았으므로, 문서에 적힌
  통계 기반 보정 절차가 아직 실제 RTL 값에 적용되지 않은 상태다.

### 4.3 Stuck

| 채널 | 활성 프레임 | 최장 연속 |
|---|---:|---:|
| gyro_z | 101 | 93 |
| temperature | 1,030 | 1,030 |
| humidity | 1,030 | 1,030 |
| lux | 1,030 | 1,030 |
1. 
- temperature/humidity/lux는 `CHANNEL_TYPE_2=0`이므로 항상 testable이다.
- 정상 캡처에서 temperature와 humidity는 각각 30초 동안 동일하고, 한 번 바뀐 뒤 다시
  동일하다. 시뮬레이션 환경값이 고정인 것이 정상인데 센서 고착으로 판단한다.
- 이 세 채널은 Stuck만 있고 Consistency가 없어서 상태는 DEGRADED에 머무르지만,
  위험도 제어를 지속적으로 보수화한다.
- gyro_z는 값이 우연히 같은 것만으로 INVALID가 된 것이 아니라, Stuck과 Consistency가
  동시에 켜지는 구간이 생겨 INVALID가 된다.

### 4.4 Consistency

| 채널 | 활성 프레임 |
|---|---:|
| accel_x | 1,034 |
| accel_y | 714 |
| gyro_z | 858 |

- 현재 프로젝트의 AXI 입력은 measurement와 Python/CARLA reference를 함께 보내지 않는다.
- `preprocessor.sv`가 속도/기울기/조향으로 reference를 내부 생성하고
  `sensor_reliability.sv`가 이 값과 measurement를 비교한다.
- 이는 이전에 정한 “CARLA와 Python이 계산한 reference를 measurement와 함께 FPGA로
  전달”하는 최종 구조와 다르다.

#### gyro 스케일/비트폭 문제

- gyro measurement 단위는 rad/s x 1,000이다.
- consistency checker에서는 `sensor_data * S_GYR`, S_GYR=1,024를 사용하므로 비교 좌변은
  약 rad/s x 1,024,000 규모다.
- gyro reference 1은 `(현재 각도-이전 각도) * C_GYR`, C_GYR=3,574로 계산한 뒤
  `pred_data_t`의 signed 16비트에 저장된다.
- 중간 계산값이 16비트를 넘으면 저장 시 wrap된다. 예를 들어 sequence 720 부근에서
  yaw 입력은 한 프레임에 약 0.21~0.22도(정수값 21~22) 변한다. 계산 결과
  `21*3574=75054`, `22*3574=78628`은 signed 16비트에 들어가지 않아 각각 9,518과
  13,092로 wrap된다. 실제 재생에서도 이 두 값이 관찰됐다. 같은 구간의 gyro_z
  measurement는 약 75이므로 비교 좌변 `75*1024=76800`도 16비트 범위를 넘는다.
- 따라서 `abs(sensor_data*1024 - pred_data) <= 270` 조건을 정상 주행에서 만족하기 어렵다.
- 이 항목은 threshold 조절만으로 해결하면 안 된다. reference의 단위/Q-format과 내부
  비트폭을 먼저 맞춰야 한다.

## 5. Timeout 확인 결과

- `UPDATE_CLK_X2=(2*CLK_FREQ_HZ)/SAMPLE_RATE_HZ`이므로 20 Hz에서 100 ms마다 timeout
  증거가 한 번 쌓인다.
- `TIMEOUT_N=10`이므로 약 1초 동안 새 sample commit이 없을 때 timeout이 확정된다.
- 새 `valid_s1`이 들어오면 timeout 증거가 2씩 감소한다.
- 고장 주입 결과 9번째 100 ms 구간까지 timeout=0, 10번째에 timeout=1로 통과했다.
- 정상 캡처의 host gap 최대값은 148.36 ms였지만 한 번뿐이고 연속 1초 누락이 아니므로
  timeout 오탐은 0건이었다.

## 6. 지금 바로 RTL 파라미터를 바꾸지 않은 이유

- 사용자 요구대로 현재 값이 CARLA 데이터에서 어떻게 동작하는지를 먼저 측정했다.
- 단일 정상 캡처만으로 임계값을 최종 확정할 수 없다.
- Jump/Noise는 정상 분포의 percentile/표준편차가 필요하고, Stuck은 정상 환경별 최장
  동일 구간이 필요하다.
- Consistency는 임계값보다 먼저 measurement/reference 인터페이스와 Q-format을
  결정해야 한다.

## 7. 권장 수정 순서

1. **reference 인터페이스 확정**
   - 최종 목표대로 Python/CARLA reference를 FPGA로 보낼지, PL 내부 동역학 모델을
     사용할지 하나로 고정한다.
   - Python reference 방식이면 AXI 주소폭/레지스터 맵을 확장하고 reference frame에도
     동일 sample_seq를 붙인다.
2. **동일 단위와 Q-format 적용**
   - measurement와 reference의 각 채널 비트폭/Q-format을 동일하게 만든다.
   - residual은 signed 확장 후 입력보다 최소 2비트 넓게 계산한다.
   - 현재 `pred_data_t`의 12/13/16비트 저장으로 인한 wrap 가능성을 제거한다.
3. **정상 데이터셋 확대**
   - 정지, 직선 가속/감속, 회전, 장애물 접근/이탈, 날씨 전환을 각각 여러 번 캡처한다.
   - 각 샘플에 scenario와 fault 주입 여부를 명시한다.
4. **파라미터 산정**
   - Jump: 정상 residual의 percentile 또는 3-sigma로 채널별 임계값 계산
   - Noise: 정상 평균절대delta와 부호전환 분포로 계산
   - Stuck: 정상 최대 동일 구간보다 긴 시간과 독립 trigger가 동시에 있을 때만 판정
5. **고장 주입 검증**
   - bias, spike, freeze, random noise, dropout을 채널별로 Python에서 주입한다.
   - detection latency, false-positive rate, recovery latency를 자동 비교한다.
6. **합격 후 RTL 수정 및 재합성**
   - 산출된 파라미터와 확정된 reference 구조만 RTL에 반영한다.
   - 전체 회귀, 합성, implementation timing, HIL 순서로 재검증한다.

## 8. 생성/갱신한 검증 산출물

- `CARLA_FPGA_PROJECT/compare_pl_replay.py`
- `sources_1/verification/tb_carla_axi_replay.sv`
- `verification_reports/pl_replay_results.csv`
- `verification_reports/pl_replay_comparison_20260813_013323.md`
- `verification_reports/pl_replay_diagnostics_20260813_013323.csv`
- `verification_reports/pl_capture_analysis_20260813_013323.json`
- `verification_reports/full_verification_rerun.log`

이번 단계에서는 검증 도구와 보고서만 변경했으며, 실제 PL 동작 코드와 파라미터는
변경하지 않았다.
