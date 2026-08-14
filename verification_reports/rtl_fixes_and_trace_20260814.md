# RTL 수정(1-1, 2-1), 온도 스케일 수정(2-2), 클럭 단위 추적 도구

2026-08-14 / FPGA 보드 없이 수행

## 0. 회귀 결과

| 검증 | 결과 |
|---|---|
| `tb_pl_full_verification` | **88 PASS / 0 FAIL** |
| `tb_risk_reliability_matrix` | **121 PASS / 0 FAIL** (기존 115 + 신규 6) |
| `tb_carla_axi_replay` | 5 samples / 0 FAIL |
| `verify_sensor_noise.py` | **10개 항목 PASS** (기존 9 + 신규 1) |
| `test_scenario_pl_alignment` | **17 PASS** |

---

## 1. (2-2) 온도 스케일 — 사용자 지적이 맞았다

`risk_types.sv`의 노면 분류가 결정적 증거다.

```systemverilog
if (sensor_data_in.temperature <= -50 && sensor_data_in.humidity >= 90)
    road_risk_A = 2'b11; // Black Ice
else if (sensor_data_in.temperature <= 0 && sensor_data_in.humidity >= 70)
    road_risk_A = 2'b10; // Ice
```

RANGE는 -500..600이다. LSB가 0.1 degC여야 **-50.0 ~ 60.0 degC**라는 물리적으로
타당한 범위가 되고, Black Ice 임계 `-50`은 **-5.0 degC**가 된다. LSB가 1 degC면
범위는 -500~600 degC, Black Ice 임계는 -50 degC로 물리적으로 도달 불가능하다.

그런데 `fpga_interface.py`는 `_quantize_signed(temperature, 1.0, 11)`, 즉
**scale 1.0**을 쓰고 있었다.

### 증상

- 실제 -3 degC(블랙아이스 조건)를 보내면 PL은 raw -3, 즉 **-0.3 degC**로 읽는다.
  Black Ice 임계 -50에 한참 못 미쳐 **영원히 BLACK ICE가 되지 않는다.**
- 그래서 codex는 `sensor.temperature = -60.0`이라는 물리적으로 불가능한 값을
  주입해 raw -60을 만들어 임계를 넘겼다. 스케일 버그를 우회한 것이다.
- WET(12 degC)/ICE(-5 degC)는 raw 12 / -5로도 `<= 0`, `humidity >= 70` 조건을
  우연히 만족해 정상 동작했다. 그래서 문제가 드러나지 않았다.

### 수정

| 항목 | 이전 | 이후 | 근거 |
|---|---|---|---|
| `fpga_interface` 온도 scale | 1.0 | **10.0** | LSB 0.1 degC |
| BLACK ICE 주입 온도 | -60.0 degC | **-8.0 degC** | raw -80 <= -50, range 내 |
| `_range_value` 온도 | 700.0 | 70.0 | raw 700 > 600 |
| `_jump_amplitude` 온도 | 12.0 | 1.2 | raw 12 > JUMP_THRESHOLD(5) |
| `_noise_amplitude` 온도 | 6.0 | 0.6 | raw 6 > NOISE_THRESHOLD_1(2) |
| `apply_sensor_conditions` triangle | `+triangle` | `+triangle*0.1` | 1 LSB 유지 |
| `sensor_noise` 온도 진폭 | 1.4 degC | 0.14 degC | 1.4 LSB 유지 |

-60.0 degC를 그대로 두면 raw -600이 되어 range 하한(-500)을 벗어나 **온도
range fault**가 되므로 반드시 함께 고쳐야 했다.

### 부수 발견: 잡음원 중복

`apply_sensor_conditions`의 `triangle`과 `SensorNoiseModel`이 같은 채널에
동시에 잡음을 넣으면, NOISE_THRESHOLD_1이 2 LSB로 가장 빡빡한 온도에서 여유가
사라진다(합산 18.5 / 한계 20). `WorldScenarioController.driven_channels`를
추가해 시나리오가 직접 구동 중인 채널은 기본 잡음에서 제외한다. 둘 중 하나만
적용된다.

### 검증 (verify_sensor_noise [8])

DRY/WET/ICE/BLACK ICE 및 CLEAR/RAIN/SNOW 프리셋 6종이 모두 의도한 tier로
분류되고, 전 프리셋이 range 안에 있다. 22.0 degC -> raw 220 확인.

---

## 2. (1-1) gyro Q-format — 비트폭은 고쳤고, 임계값 문제가 새로 드러났다

### 2.1 산술 확인: C_GYR = 3574는 옳다

```
Δincline_raw = yaw_rate[rad/s] × (180/π) × 100 / 20 = yaw_rate × 286.479
우변 = Δincline_raw × C_GYR = yaw_rate × 286.479 × 3574 = yaw_rate × 1,023,876
좌변 = gyro_raw × S_GYR     = yaw_rate × 1000 × 1024   = yaw_rate × 1,024,000
```

비율 0.99988. **스케일 상수는 정확하다.** 문제는 저장 폭뿐이었다.

### 2.2 수정

`pred_gyro_x_1 / y_1 / z_1`을 `signed [15:0]` -> **`signed [27:0]`**,
`u_cons_1_gyro_x/y/z`의 `WIDTH`를 16 -> **28**, `sensor_data`는 명시적 부호확장.

28비트 근거:
- 117 deg/s 선회: 585 × 3574 = 2,090,790 (22비트)
- yaw가 ±180 deg 경계를 넘는 순간: 36000 × 3574 = 128,664,000 (28비트)

16비트일 때 실제 값 511,082는 **-13,206으로 wrap**되어 부호까지 뒤집혔다.

### 2.3 클럭 추적으로 확인한 결과

`make_trace_vectors.py turn` (0.5 rad/s 정상 선회 60표본) -> `run_pl_trace.bat`
-> `analyze_pl_trace.py --gyro`:

```
  cycle   gyro_z    좌변(x1024)     우변(pred)       잔차     판정
    218      499        510976         511082       -106      OK
    496      496        507904         514656      -6752     초과
   1330      503        515072         511082       3990     초과

부호 반전(wrap) 징후 없음. -> 비트폭은 충분하다.

임계값 타당성
  잔차 최대 : 6752
  잔차 평균 : 2829
  현재 TH_GYR: 270
  기준값 계단: [3574]
```

**비트폭 문제는 해결됐다**(양변이 같은 부호로 511,000 부근에서 함께 움직임).

### 2.4 새로 드러난 문제 — 임계값이 양자화 바닥보다 작다

기준값 `pred`는 incline 1 LSB(0.01 deg) 단위로만 움직이므로 **계단 크기가
3574**다. 따라서 양자화만으로 항상 **±1787**의 잔차가 생긴다.

**TH_GYR = 270 은 이 양자화 바닥의 1/6에 불과하다.** 비트폭을 고쳐도 선회 중
gyro consistency는 구조적으로 통과할 수 없다.

해결 선택지 (파라미터 결정이 필요해 이번에 변경하지 않음):

| 안 | 내용 | 비고 |
|---|---|---|
| A | TH_GYR을 4000~7300으로 상향 | 가장 단순. 검출 민감도 4000/1024 = 0.25 deg/s로 여전히 충분 |
| B | incline 해상도를 0.001 deg로 | 16비트 struct 확장 필요, 범위 ±180000 = 19비트 |
| C | 기준값을 여러 표본 창으로 계산 | 상대 양자화 오차 감소, preprocessor 로직 변경 |

**권장: A.** 실측 잔차 최대 6752 + 여유 -> **TH_GYR = 7300** 정도.
다만 이 값은 실제 캡처 기반으로 확정하는 게 맞아 Phase 1 파라미터 산정
단계로 넘긴다.

---

## 3. (2-1) 마찰 비례 제동 블렌딩

### 이전

```systemverilog
dangerous_brake_situation = (eff_tier_road_A >= 2'b10) || (eff_tier_posture_C >= 2'b10);
if (dangerous_brake_situation) final_brake = 4'd0;
else                          final_brake = get_max4(col_brake, road_B_brake);
```

저마찰/횡방향 위험이면 제동을 **0으로 강제**해, 동시에 발생한 충돌 EMERGENCY의
brake 10 요청까지 지웠다.

### 이후

요청 제동을 0으로 만들지 않고 노면이 견딜 수 있는 **상한으로 제한**한다.

```systemverilog
localparam logic [3:0] BRAKE_CAP_ICE       = 4'd5;
localparam logic [3:0] BRAKE_CAP_BLACK_ICE = 4'd3;
localparam logic [3:0] BRAKE_CAP_LATERAL   = 4'd5;
localparam logic [3:0] BRAKE_CAP_NONE      = 4'd15;

brake_cap       = get_min4(surface_brake_cap, lateral_brake_cap);
requested_brake = get_max4(col_brake, road_B_brake);
final_brake     = get_min4(requested_brake, brake_cap);
```

상한 4개는 이름 붙은 localparam 한 곳에 모았다. 마찰계수 실측 후 이 값만
조정하면 된다.

### 검증 (신규 6건, 모두 PASS)

| 상황 | 요청 | 상한 | 최종 |
|---|---|---|---|
| DRY + EMERGENCY | 10 | 15 | **10** (변화 없음) |
| WET + EMERGENCY | 10 | 15 | **10** (변화 없음) |
| ICE + EMERGENCY | 10 | 5 | **5** (이전 0) |
| 횡방향 DANGER + EMERGENCY | 10 | 5 | **5** (이전 0) |
| BLACK ICE + 횡방향 DANGER + EMERGENCY | 10 | 3 | **3** (낮은 상한 승) |
| BLACK ICE, 충돌 없음, rough road | 2 | 3 | **2** (이전 0) |

기존 정책을 인코딩하고 있던 시험 4건을 새 정책 값으로 갱신했다
(`ice brake suppression` 0->1, `lateral danger` 0->1,
`combined emergency+black-ice` 0->3, `black ice brake suppression` 0->3).

---

## 4. 클럭 단위 추적 도구 (신규)

FPGA 판단의 중간 과정을 클럭 단위로 전부 보기 위한 도구다. 보드 불필요.

```
make_trace_vectors.py   시나리오 -> AXI 벡터 CSV (CARLA 없이)
        |
run_pl_trace.bat        xsim으로 재생, 매 클럭 46개 신호를 CSV 1행으로 기록
        |
analyze_pl_trace.py     사람이 읽는 형태로 분석
```

### 기록 단계

| 단계 | 내용 |
|---|---|
| S0 | AXI 커밋 sample_seq |
| S1 | preprocessor: 입력 스냅샷, 1차 차분(delta_*), 동역학 기준값(pred_*) |
| S2 | 채널별 range/jump/stuck/noise/timeout 확정 비트맵, timeout phase 카운터 |
| S2b | 관계식별 consistency 확정 비트 |
| S3 | 채널별 NORMAL / DEGRADED / INVALID |
| S4 | 원시 위험도 tier (collision, road_A/B, vision_A, posture_C) |
| S5 | 유효 tier, col_brake, road_B_brake, 상한 3종, 요청/최종 제동, TD/MRM/HUD |

### 분석 명령

```
python analyze_pl_trace.py             표본별 판단 요약
python analyze_pl_trace.py --changes   신호가 바뀐 클럭만 (언제 판단이 바뀌었나)
python analyze_pl_trace.py --clocks 300 340   구간 원시 덤프
python analyze_pl_trace.py --gyro      gyro consistency 양변 + 임계값 타당성
python analyze_pl_trace.py --brake     제동 중재 단계별 추적
```

파형이 필요하면 `run_pl_trace.bat <vectors.csv> vcd` -> `pl_trace.vcd`
(전 계층 덤프이므로 긴 캡처에서는 파일이 크다).

---

## 5. 변경 파일

**RTL** (이번에 처음으로 수정)
- `types_pkg.sv` — pred_gyro_*_1 16 -> 28비트
- `sensor_reliability.sv` — u_cons_1_gyro_x/y/z WIDTH 28, 부호확장
- `risk_control.sv` — 마찰 비례 제동 블렌딩

**Testbench**
- `tb_pl_full_verification.sv` — 제동 정책 시험 1건 갱신
- `tb_risk_reliability_matrix.sv` — 3건 갱신 + 6건 신규
- `tb_pl_trace.sv` (신규), `run_pl_trace.bat` (신규)

**Python**
- `fpga_interface.py` — 온도 scale 10.0
- `world_scenario_controller.py` — BLACK ICE -8.0 degC, triangle 0.1배, driven_channels
- `control_panel.py` — 온도 주입값 4종 재조정
- `sensor_noise.py` — 온도 진폭 0.14 degC
- `main.py` — skip에 driven_channels 합류
- `test_scenario_pl_alignment.py` — 노면 tier 시험이 raw로 양자화 후 비교
- `verify_sensor_noise.py` — [8] 노면 분류 검증 추가
- `analyze_pl_trace.py`, `make_trace_vectors.py` (신규)

## 6. 보드 확보 후 확인할 것

- 재합성 후 timing (pred_gyro 28비트 확장이 경로에 미치는 영향)
- 저마찰 제동 블렌딩의 실제 차량 거동
- TH_GYR 확정을 위한 실주행 잔차 분포 측정
