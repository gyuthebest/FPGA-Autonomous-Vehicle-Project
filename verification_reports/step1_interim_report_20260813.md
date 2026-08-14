# PL 검증 1단계 중간 보고서 (2026-08-13)

## 1. 현재 결론

- 사용자가 선택한 PL 클럭 목표는 90 MHz이다.
- Zynq UltraScale+ PS가 해당 설정에서 실제 생성하는 PL0 클럭은 88.888 MHz이다.
- 전체 Block Design을 이 실제 클럭에 맞춰 새로 합성·배치배선하고 bitstream/XSA까지 생성했다.
- 최종 timing은 setup/hold 모두 통과했다.
- PL 전체 self-checking 회귀는 75 PASS / 0 FAIL이다.
- FPGA 보드가 없는 상태의 CARLA 20 Hz 실시간 데이터 캡처를 시작했다.

## 2. 클럭 및 timeout 기준

- PS 요청 주파수: 90 MHz
- 실제 구현 주파수: 88.888 MHz
- 실제 클럭 주기: 11.250 ns
- CARLA/PS sample rate: 20 Hz
- sample period: 50 ms
- `UPDATE_CLK_X2=(2*CLK_FREQ_HZ)/SAMPLE_RATE_HZ`
- `UPDATE_CLK_X2` 시간 의미: 100 ms
- timeout 의도 B: valid가 계속 없으면 100 ms마다 timeout evidence를 1회 누적
- `TIMEOUT_N=10`일 때 confirmed timeout까지 약 1초
- valid가 도착하면 timeout phase를 즉시 초기화하고 healing을 적용

## 3. 적용한 변경

### 3.1 실제 88.888 MHz 반영

다음 기본값과 packaged-IP parameter를 `88_888_000`으로 맞췄다.

- `sources_1/new/top_controller.sv`
- `sources_1/new/sensor_reliability.sv`
- `sources_1/new/risk_control.sv`
- `sources_1/new/mask_20s.sv`
- `component.xml`

Block Design에서는 PS의 요청값 90 MHz와 실제값 88.888 MHz가 모두 확인된다.

### 3.2 timeout 의도 B 반영

- `sensor_reliability.sv`에 모든 센서 채널이 공유하는 periodic timeout phase counter를 두었다.
- phase counter는 valid 도착 시 0으로 초기화된다.
- valid가 없으면 매 100 ms 경계마다 모든 채널에 timeout evidence가 들어간다.
- `sensor_checker.sv`의 raw timeout은 `!valid_s1`로 gating했다.

기존 구현은 카운터가 특정 값과 같아지는 단 한 클럭에만 raw timeout을 발생시켜, valid가 영원히 오지 않아도 TIMEOUT_N까지 누적되지 못하는 문제가 있었다.

### 3.3 장애물 출현 event 수정

Python에서 다음 조건일 때만 `situation=3'b001`을 생성한다.

```text
previous_distance >= 200.0 m AND current_distance < 200.0 m
```

즉, CARLA radar의 no-target sentinel 200 m에서 실제 탐지 거리로 전환되는 순간만 장애물 출현으로 처리한다. 이 event 생성은 Python에 있고 PL은 AXI REG8로 전달된 값을 사용한다.

### 3.4 Radar 접근속도 부호 정합

CARLA C++은 다음과 같이 상대 방사속도를 계산한다.

```text
dot(target_velocity - ego_velocity, target_direction)
```

따라서 전방 물체에 가까워지는 상황은 CARLA raw radar에서 음수이다. 반면 PL의 `risk_types.sv`는 `approach_speed > 0`을 접근으로 해석한다. Python–PL 경계에서 다음처럼 부호를 반전했다.

```text
PL approach_speed = -CARLA radar detection.velocity
```

이 변경 전에는 실제 접근 중 충돌 위험 분기가 실행되지 않을 수 있었다. PL RTL의 비트폭이나 임계값은 변경하지 않았다.

### 3.5 자동 검증 실행 보완

- `CARLA_MAP=Town04` 환경 변수가 있으면 맵 선택 표준입력을 기다리지 않는다.
- 환경 변수가 없으면 기존 대화형 맵 선택 메뉴가 유지된다.
- `run_pl_capture_no_fpga.ps1`은 FPGA 비활성, 20 Hz logging, Town04를 자동 설정한다.

## 4. Vivado 전체 빌드 결과

- 합성: PASS
- 구현: PASS
- bitstream 생성: PASS
- XSA 생성: PASS
- timing setup: WNS +0.280 ns, TNS 0 ns, failing endpoint 0
- timing hold: WHS +0.007 ns, THS 0 ns, failing endpoint 0
- LUT: 7,950 / 47,232 (16.83%)
- FF: 6,216 / 94,464 (6.58%)
- DSP: 24 / 240 (10.00%)
- BRAM: 0

생성물:

- `FPGA_project/FPGA_project.runs/impl_1/design_1_wrapper.bit`
- SHA-256: `DA2808268B92FE22EFDE76EE2A97D22F6101FC6409AE6E055375B81A2152E9AA`
- `FPGA_project/design_1_wrapper.xsa`
- SHA-256: `3A77BE5970A35D603C9D953FC1A074DA554F7F8C68EAFE3358EF0C4C6E192C1E`

이전 100 MHz 생성물은 `verification_reports/pre_88888mhz_artifacts`에 보관했다.

## 5. DRC 해석

- error: 0
- critical warning: 0
- warning: 67
- warning 종류: DPIP-2 26건, DPOP-3 17건, DPOP-4 24건

67건은 DSP48 내부 input/output pipeline 사용을 권고하는 성능 경고이다. 현재 88.888 MHz에서는 timing을 만족하므로 기능 오류가 아니다. DSP pipeline을 추가하면 latency와 valid 정렬이 바뀌므로 현재는 수정하지 않았다.

## 6. 회귀 검증 결과

- Python compile: PASS
- PL 전체 self-checking testbench: 75 PASS / 0 FAIL
- CARLA AXI vector smoke replay: 5 samples / 0 FAIL
- root RTL과 packaged-IP 내부 RTL SHA 일치 확인: PASS
- Vivado IP status 6개: 모두 Up-to-date / No changes required

## 7. 첫 정상 로그 분석

분석 파일: `CARLA_FPGA_PROJECT/logs/pl_verification/pl_capture_20260813_013323.csv`

- sample: 1,045개
- sequence 누락: 없음
- nominal period: 50 ms
- host gap 평균: 약 50.73 ms
- host gap p95: 약 52.66 ms
- host gap 최대: 약 148.36 ms
- IMU frame lag: 전 sample 0
- Radar frame lag: 전 sample 0
- fault label: 전 sample `none`

중요 관찰:

- 첫 sensor sample 한 개에서 Radar/IMU 비정상 startup 값이 관측됐다.
- 이후 sample은 정상 범위로 복귀했다.
- 현재 range debounce N=3과 jump 첫 2 sample masking 때문에 confirmed fault로 이어지지는 않는다.
- startup 값을 명시적으로 버릴지는 파라미터 조정과 분리해서 최종 결정해야 한다.
- 정상 주행에서도 현재 distance jump threshold 100 raw를 넘는 hit가 다수 발생했다. 다만 이 capture만으로는 도로 구조물 radar point 전환과 실제 센서 fault를 분리할 수 없으므로 아직 parameter를 수정하지 않았다.

## 8. 현재 진행 중인 단계

CARLA Town04가 FPGA 없는 검증 모드로 실행 중이며 아래 로그가 20 Hz로 증가하고 있다.

- `CARLA_FPGA_PROJECT/logs/pl_verification/pl_capture_20260813_014151.csv`
- `CARLA_FPGA_PROJECT/logs/pl_verification/pl_vectors_20260813_014151.csv`

이 로그에는 부호 수정된 접근속도가 들어간다. 첫 startup sample 이후 실제 접근 상황에서 양수로 기록되는 것을 확인했다.

## 9. 다음 검증

1. 정상 주행에서 정지, 가속, 감속, 좌회전, 우회전, 장애물 접근·이탈 구간을 확보한다.
2. capture 종료 후 채널별 range, jump, stuck, noise 통계를 산출한다.
3. 동일 REG0..REG9를 AXI replay testbench에 입력한다.
4. PL reliability/risk 출력과 상황 label을 비교한다.
5. 오탐·미탐 근거가 확보된 parameter만 수정 후보로 보고한다.
6. 사용자가 의도를 확인한 뒤 parameter 또는 PL 로직을 수정하고 전체 회귀 및 timing을 다시 실행한다.
