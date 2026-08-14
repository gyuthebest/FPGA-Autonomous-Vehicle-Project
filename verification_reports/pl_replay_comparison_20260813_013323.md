# CARLA 캡처 대 PL RTL 재생 비교 보고서

## 검증 범위

- CARLA 캡처: `C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project\CARLA_FPGA_PROJECT\logs\pl_verification\pl_capture_20260813_013323.csv`
- RTL 재생 결과: `C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project\verification_reports\pl_replay_results.csv`
- 캡처 샘플: 1045개
- 재생 샘플: 1045개
- 일치한 sample_seq: 1045개
- 누락된 재생 sequence: 0개
- 예상하지 않은 재생 sequence: 0개

이 보고서는 RTL 내부 식을 그대로 복제하지 않는다. 입력 라벨, 명령 입출력 관계, 시퀀스 정합성 및 안전 불변조건을 독립적으로 검사한다.

## 외부 안전 불변조건

- 위반 0건

## 정상 라벨 데이터에서의 관찰

- `fault_label=none` 샘플: 1045개
- 하나 이상의 INVALID 채널이 나온 샘플: 101개
- HUD 경고: 101개
- Transition Demand: 306개
- MRM: 116개
- 가속 명령 변경: 716개
- 제동 명령 변경: 126개
- 조향 명령 변경: 0개
- 제한속도 변경: 1042개

제어 개입 자체는 위험도 로직의 정상 동작일 수 있다. 하지만 이 캡처의 모든 샘플이 고장 없음으로 라벨링되어 있으므로 INVALID/TD/MRM은 오탐 후보로 분류하여 원인을 추가 확인해야 한다.

## 신뢰도 상태 집계

| 채널 | NORMAL | DEGRADED | INVALID | RESERVED |
|---|---:|---:|---:|---:|
| distance | 9 | 1036 | 0 | 0 |
| approach_speed | 0 | 1045 | 0 | 0 |
| accel_x | 10 | 1035 | 0 | 0 |
| accel_y | 186 | 859 | 0 | 0 |
| accel_z | 1035 | 10 | 0 | 0 |
| gyro_x | 1036 | 9 | 0 | 0 |
| gyro_y | 1006 | 39 | 0 | 0 |
| gyro_z | 186 | 758 | 101 | 0 |
| temperature | 5 | 1040 | 0 | 0 |
| humidity | 5 | 1040 | 0 | 0 |
| lux | 15 | 1030 | 0 | 0 |

## 검사기별 활성 샘플 수

| 검사기 | 채널 | 활성 샘플 | 최장 연속 활성 |
|---|---|---:|---:|
| jump_mask | distance | 1010 | 987 |
| jump_mask | approach_speed | 987 | 982 |
| jump_mask | accel_x | 1 | 1 |
| jump_mask | accel_y | 1 | 1 |
| jump_mask | accel_z | 1 | 1 |
| stuck_mask | gyro_z | 101 | 93 |
| stuck_mask | temperature | 1030 | 1030 |
| stuck_mask | humidity | 1030 | 1030 |
| stuck_mask | lux | 1030 | 1030 |
| noise_mask | distance | 902 | 112 |
| noise_mask | approach_speed | 1039 | 663 |
| noise_mask | accel_x | 151 | 45 |
| noise_mask | accel_y | 382 | 157 |
| noise_mask | accel_z | 10 | 10 |
| noise_mask | gyro_y | 19 | 9 |
| noise_mask | gyro_z | 142 | 115 |
| noise_mask | temperature | 10 | 10 |
| noise_mask | humidity | 20 | 10 |
| consistency_mask | accel_x | 1034 | 1034 |
| consistency_mask | accel_y | 714 | 499 |
| consistency_mask | accel_z | 3 | 3 |
| consistency_mask | gyro_x | 9 | 9 |
| consistency_mask | gyro_y | 20 | 3 |
| consistency_mask | gyro_z | 858 | 563 |

## 제어 플래그 최초/최종 sequence

| 플래그 | 최초 | 최종 | 전체 활성 샘플 |
|---|---:|---:|---:|
| hud_warning | 731 | 845 | 101 |
| transition_demand | 740 | 1045 | 306 |
| mrm | 930 | 1045 | 116 |

## 판정

- AXI 전달 및 파이프라인 sequence 정합성은 외부 불변조건 기준으로 확인한다.
- 정상 라벨 캡처에서 지속적으로 활성화된 검사기는 파라미터 또는 기준값 생성 방식의   재검토 대상이다.
- 세부 프레임은 별도 CSV에서 확인할 수 있다: `C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project\verification_reports\pl_replay_diagnostics_20260813_013323.csv`
