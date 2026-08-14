# HANDOFF — FPGA 신뢰도/위험도 기반 자율주행 로직

이 문서 하나만 읽고 바로 이어서 작업할 수 있도록 썼다.
작업 폴더는 `C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project`.
한국어로 답하고, RTL 수준 용어는 설명 없이 써도 된다.

작업 규칙은 `CLAUDE.md`에 있다. **먼저 읽어라.** 특히 5절(골든 모델 규칙)과
7절(검증 결과 보고 형식)은 이 프로젝트의 판단 기준 자체다.

---

## 1. 이 프로젝트가 하는 일

차량 센서 11채널을 받아 **① 각 센서를 믿을 수 있는가(신뢰도)**,
**② 지금 얼마나 위험한가(위험도)** 를 FPGA에서 판정하고,
그 결과로 가속/제동/조향/등화 명령을 중재해 되돌려준다.

핵심 주장은 "신뢰도 **기반** 위험도"다. 센서가 못 미더우면 위험도를
보수적으로 올린다. 예를 들어 거리 센서가 INVALID면 충돌 위험도를
마지막 유효값 + 1 단계, 최소 바닥값 이상으로 끌어올린다.

### 데이터 흐름

```
CARLA (Python, 20 Hz)
  └─ fpga_interface.build_input_words()  물리값 -> REG0..REG9 (32비트 x 10)
      └─ UDP :5001 -> A53 (ps_carla_bridge.c)
          └─ AXI 쓰기 (slv_reg0..9), reg9 = sample_seq 가 커밋 신호
              └─ PL  preprocessor -> sensor_reliability -> risk_types -> risk_control
                  └─ AXI 읽기 (read_reg9..14)
                      └─ UDP 응답 -> FPGAResult
```

`valid_s0 = (sample_seq_in != sample_seq_out)`. **reg9를 바꿔야만 PL이
새 표본으로 인식한다.** 이 성질이 timeout 시험의 열쇠다(7절 참조).

### 신호 스케일 (물리값과 raw를 혼동하면 전부 틀어진다)

| 채널 | raw LSB | range | 비고 |
|---|---|---|---|
| distance | 0.01 m | 0..20000 | **20000 = 무표적 sentinel** (200 m 측정이 아니다) |
| approach_speed | 0.01 m/s | ±4000 | AXI는 10비트<<3 (LSB 8) |
| accel_x/y/z | 0.01 m/s² | ±1600 | |
| gyro_x/y/z | 0.001 rad/s | ±16000 | |
| temperature | **0.1 °C** | -500..600 | BLACK ICE 임계 -50 = **-5.0 °C** |
| humidity | 1 % | 0..100 | |
| lux | 1 | 0..130000 | |
| speed_x/y/z | 0.01 m/s | ±8192 | AXI는 8비트<<6 + reg8 하위 6비트 |
| incline_x/y/z | 0.01 deg | ±18000 | |
| steering | full lock의 1% | ±100 | AXI는 5비트<<3 + reg8[26:24] |

---

## 2. 현재 상태

### 구현 검증 — 완료

**"FPGA가 RTL 소스대로 동작하는가" 는 확정이다.**
실주행 자극 3,509프레임을 지터 없이 재생해 **38,379건 전수 일치**.
폐루프 캡처 3,409프레임도 전수 일치.

### 검증 커버리지

| 항목 | 상태 |
|---|---|
| 신뢰도 11채널 상태 | 실보드 전수 일치 |
| 관계식 1~17 | 골든 모델 구현, RTL과 일치 (9~16은 오프라인 정차 시나리오로) |
| range/jump/stuck/noise/timeout | 일치 |
| risk 워드(유효 tier) | 99.8% (나머지는 지터 하류) |
| HUD / MRM | 100% |
| TD 잔여초 | ±1초 이내 (PL 자유 카운터 위상, 복원 불가) |
| transport timeout | 결정론적 시험 PASS |
| 라이브 시나리오 | 71 PASS / 4 FAIL (실패 4건 전부 원인 규명, RTL 무관) |

### 빌드 (현재 보드에 올라간 것)

```
setup WNS +0.311 ns   TNS 0   hold WHS +0.005 ns   THS 0
DRC error 0 / critical warning 0
LUT 18% FF 6.6% DSP 10%
```

### 정상 주행 오탐률 (고장 미주입, 잡음 ON, 3분)

| 채널 | 개루프 | 폐루프 | 주원인 |
|---|---|---|---|
| accel_x | 9.57% | 7.61% | noise 4.9% + 관계식3 4.6% |
| accel_y | 7.14% | 11.80% | noise |
| gyro_z | 2.87% | 7.38% | noise |
| lux | 6.39% | 6.40% | stuck |
| 나머지 7채널 | 0~2.8% | 0~2.8% | |

**INVALID 오탐은 개루프·폐루프 모두 0건.**

### 이 오탐은 "정상"이 아니다 — 남은 최대 결함

고장이 하나도 없는 주행에서 채널당 최대 **12표본 중 1표본꼴**로 DEGRADED가
뜬다. 이건 알고리즘이 아직 덜 다듬어졌다는 뜻이지 정상 동작이 아니다.

**무해하지도 않다.** `risk_control.calc_effective_tier`는 신뢰도가 DEGRADED면
위험도 tier를 한 단계 올린다(`pre = min(raw+1, N-2)`, `eff = max(raw, pre)`).
따라서 accel_y 오탐 → `Re_posture_C` DEGRADED → 횡방향 tier 상승 →
**가속 상한이 걸린다**(tier 1이면 accelerator ≤ 7). 오탐이 실제 차량 제어를
흔든다. *(tier 상승 논리는 보드와 risk 워드 99.8% 일치로 확인됨. 이 캡처들에서
실제 가속 명령이 얼마나 깎였는지는 아직 세어 보지 않았다.)*

원인 분해 (잡음 OFF, 동일 궤적 3,416표본):

| 원인 | 채널 | 비율 |
|---|---|---|
| `NOISE_THRESHOLD_2` 부호 반전 7회 초과 | accel_y | 11.5% |
| 〃 | accel_x | 5.6% |
| 〃 | gyro_z | 4.7% |
| 〃 | accel_z | 2.7% |
| 관계식 3 (accel_x 동역학 기준값) | accel_x | 4.6% |
| 관계식 7 (gyro_y) | gyro_y | 2.6% |
| 관계식 8 (gyro_z) | gyro_z | 1.9% |

`NOISE_THRESHOLD_1`(accel 500 → 10창 합 5000 = 50 m/s²)은 현실적으로
도달하지 않으므로 noise 오탐은 **전부 부호 반전 쪽**이다.

두 갈래로 나뉜다.

1. **부호 반전 임계 7회가 진동하는 가속도계 신호에 부적합하다.**
   실제 IMU는 평균 주변에서 매 표본 부호가 뒤집힌다. 이 판정은 "잡음이
   심하다"가 아니라 "정상 IMU다"를 잡아내고 있다.
2. **관계식 3/7/8의 기준값이 실제 동역학을 못 따라간다.**
   관계식 3은 속도를 2표본 차분해 가속도를 추정하는데, 급가감속 구간에서
   실제 가속도와 벌어진다(잔차 최대 134, 임계 76).

둘 다 파라미터 변경이라 **승인 없이 손대면 안 된다.** 8절 2번·8번 참조.

### 고장 검출 (52건 주입)

검출 51건 (중앙 1표본 = 50 ms, 최대 18표본), 복구 중앙 11표본 / 최대 58표본.
미검출 1건 `approach_speed:stuck`, 과도만 1건 `distance:stuck` — 둘 다
교차 게이팅 사각지대(8절 1번).

### 노이즈 강건성 (통제 실험, 동일 궤적 3,416표본)

| 채널 | OFF | x1 | x2 | x3 | x4 |
|---|---|---|---|---|---|
| accel_x | 10.19% | 8.28% | 6.41% | 5.71% | 5.68% |
| accel_y | 11.50% | 7.76% | 4.48% | 2.87% | 2.58% |
| temperature | 0.00% | 0.00% | **13.67%** | 42.62% | 66.80% |
| humidity | 0.00% | 0.00% | **19.70%** | 48.33% | 77.20% |
| lux | 0.00% | 0.00% | 0.00% | 0.00% | 0.00% |
| 전체 평균 | 2.92% | 2.43% | 4.93% | 10.04% | 14.86% |

- **강건성 한계 x2.0.** 정하는 것은 온습도 `NOISE_THRESHOLD_1 = 2`다.
  바닥 진동이 10창 합 10을 쓰고 한계가 20이라 여유가 10뿐이다.
- **INVALID은 x4까지 전 배율 0건.** 위험도 판정이 멈추지 않는다.
- 운동 채널은 잡음이 늘수록 오탐이 **준다**(원인은 8절 2번).

**배율마다 CARLA를 다시 주행시키지 마라.** 그렇게 하면 accel_x가
10.19 → 14.92 → 6.08 → 20.25%로 단조롭지 않게 나온다. autopilot이 같은
궤적을 보장하지 않아 잡음 효과와 주행 차이가 섞인다. 반드시
`noise_injection_sweep.py`(무잡음 캡처를 기준 궤적으로 고정하고 계측 잡음만
재주입)를 써라.

**남은 절반**: 배율별 고장 주입으로 **검출이 유지되는지** 확인해야 한다.
강건성은 "오탐이 안 늘고 검출은 유지"여야 의미가 있다. 다만 x2부터
온습도 배경 잡음이 임계에 닿으므로 그 배율에서 온습도 noise 검출은
변별력을 잃는다는 것은 이미 확정이다.

### 이 표의 검증 상태 — 반드시 구분해서 읽어라

| 항목 | 보드 확인 | 비고 |
|---|---|---|
| 바닥 진동 효과 (온습도·조도 0.00%) | **O** | 캡처 `170103` 실측 |
| 스윕 4회 주행 캡처 자체 | **O** | 전부 `FPGA_ENABLED=1` |
| timeout 시험 | **O** | 보드/모델 11/11 INVALID |
| **위 통제 실험 표 (OFF~x4)** | **X** | **전부 골든 모델 계산이다** |
| 스윕 캡처의 지터 없는 재생 대조 | **X** | 미실행 |

**통제 실험 표는 보드 측정이 아니다.** 같은 궤적에 잡음만 바꾸려면 CARLA를
다시 돌릴 수 없어 오프라인 재주입을 썼고, 판정은 골든 모델이 했다.
모델이 지터 없는 자극에서 보드와 비트 일치한다는 근거는 있으나, 이 숫자
자체를 "보드에서 측정했다"고 말하면 안 된다.

부분 교차 검증은 됐다. 배율마다 따로 주행한 캡처(궤적은 다름)와 비교하면

| 항목 | 보드 캡처 | 모델 통제실험 |
|---|---|---|
| temperature x2 | 13.78% | 13.67% |
| temperature x3 | 42.92% | 42.62% |
| humidity x2 | 19.69% | 19.70% |
| **lux x1** | **6.38%** | **0.00%** |

온습도는 근접한다. **lux만 어긋나고 원인이 규명되지 않았다.** 주행이 달라
조도 조건(그림자·시간대)이 달랐을 가능성이 크지만 확인해야 한다.
→ 8절 9번.

### 환경 채널 바닥 진동 (중요)

온도/습도/조도는 `CHANNEL_TYPE_2=0`이라 항상 testable이고 `STUCK_N=15`라,
값이 일정하면 15표본 만에 stuck이 확정된다. 실측: 잡음 OFF 개루프 3분에서
**temperature 100% / humidity 100% / lux 93.3%가 DEGRADED**였다.

그래서 `sensor_noise.py`에 **바닥 진동**을 넣었다. 계측 잡음과 달리
`CARLA_SENSOR_NOISE=0`에서도 적용되고 `SCALE` 배율도 받지 않는다.
주기 8 삼각파(raw −2,−1,0,1,2,1,0,−1), 물리 진폭 온도 ±0.2 °C / 습도 ±2 % /
조도 ±2. 매 표본 `|delta| = 1`이라 stuck이 안 쌓이고, 10창 부호 반전 2회
(한계 7) · 10창 `|delta|` 합 10(한계 20)으로 다른 검사기는 건드리지 않는다.
도입 후 세 채널 오탐률 **0.00%**.

---

## 3. 검증 체계 — 이 프로젝트에서 가장 중요한 부분

두 질문을 **절대 섞지 마라.**

### 질문 A. "FPGA가 알고리즘을 그대로 구현했는가"

골든 모델 `pl_model.py`가 담당한다. RTL을 전사한 것이 아니라 각 신호의
정의(단위·임계·디바운스)를 근거로 다시 구현했고, 임계값 상수만 공유한다.

```
pl_model.py            PL 파이프라인의 비트 단위 Python 구현
compare_golden_vs_pl.py  모델 vs (RTL 시뮬 | 실보드) 대조
```

불일치가 나오면 원인은 셋 중 하나이며 **반드시 구분해야 한다.**

1. RTL이 의도와 다르게 구현됨
2. 합성/타이밍/AXI 전송 문제
3. 골든 모델이 사양을 잘못 옮김

구분법: 같은 벡터를 xsim에 재생한다. 시뮬은 맞는데 보드만 틀리면 2번,
시뮬도 틀리면 1번 또는 3번.

**불일치가 났다고 골든 모델을 RTL에 맞추지 마라.** 어느 쪽이 옳은지
먼저 판정해야 한다. 2026-08-14에 이 규칙 덕분에 RTL 결함 3건을 찾았다.

### 질문 B. "알고리즘 자체가 옳은가"

골든 모델은 이 질문에 **답하지 못한다.** 모델과 RTL이 같은 오해를
공유할 수 있기 때문이다. B는 다음으로 판단한다.

- 물리 (예: 횡가속도에 구심항 v·ω가 빠졌다 → 잔차 111→7)
- 산술 (예: Verilog 곱셈 폭 규칙 위반으로 wrap)
- 고장 주입 라벨 (52건 중 몇 건을 잡는가)
- 안전 불변조건 (정상 주행에서 INVALID가 뜨면 안 된다)
- 양자화 바닥 (기준값 계단 > 임계값이면 구조적으로 통과 불가)

### 모델 독립성의 한계 (정직하게)

2026-08-14에 관계식 17건을 완성하면서 pred 수식·LUT·임계값을 RTL에서
읽어 옮겼다. **모델과 사양 문서(구글 독스 표1, (최신)consistency check)의
대조는 아직 안 됐다.** 상태 판정식은 사용자가 구조적 방식으로 확인했다.

---

## 4. 도구

전부 `CARLA_FPGA_PROJECT/` 안에 있고 표준 라이브러리만 쓴다(carla 제외).

| 파일 | 하는 일 |
|---|---|
| `pl_model.py` | 골든 모델. 관계식 1~17, 검사기 5종, 유효 tier, TD/MRM |
| `compare_golden_vs_pl.py` | 모델 vs 시뮬/보드 대조. 지터 상관까지 보고 |
| `board_smoke_test.py` | UDP 왕복, 캡처 재생, timeout 시험 |
| `pl_capture_metrics.py` | 오탐률, 검사기별 확정률, 관계식별 잔차 분포 |
| `fault_latency_metrics.py` | 검출/복구 지연, 미검출·과도만 구분 |
| `noise_injection_sweep.py` | **통제 실험**: 같은 궤적에 잡음만 배율 변경 |
| `noise_robustness_report.py` | 서로 다른 캡처들을 나란히 비교 (궤적이 다르면 해석 주의) |
| `make_trace_vectors.py` | CARLA 없이 시나리오 벡터 생성 |
| `analyze_pl_trace.py` | xsim 트레이스를 클럭 단위로 분석 |
| `live_scenario_verifier.py` | 제어판 조건을 프로그램으로 주입 (75건) |

### 자주 쓰는 명령

```bash
sources_1\verification\run_offline_verification.bat
```
보드 없이 전체 회귀. **OVERALL: PASS** 여야 한다. 무엇을 바꾸든 이걸 통과시켜라.

```bash
python CARLA_FPGA_PROJECT\board_smoke_test.py --frames 30
```
보드 UDP 왕복 + 골든 모델 즉시 대조.

```bash
python CARLA_FPGA_PROJECT\board_smoke_test.py --replay <capture.csv>
```
**비트 정확성 판정은 반드시 이걸로 한다.** 이유는 7절 함정 2번.

```bash
python CARLA_FPGA_PROJECT\board_smoke_test.py --timeout-test
```
transport timeout 결정론적 시험.

```bash
python CARLA_FPGA_PROJECT\pl_capture_metrics.py <capture.csv> --json out.json
python CARLA_FPGA_PROJECT\fault_latency_metrics.py <capture.csv>
python CARLA_FPGA_PROJECT\compare_golden_vs_pl.py --board <capture.csv>
```

CARLA 주행 (개루프):
```
set FPGA_ENABLED=1 & set CARLA_MAP=Town04 & set PL_VERIFY_LOG=1
set CARLA_APPLY_FPGA=0 & set CARLA_RUN_SECONDS=180
python CARLA_FPGA_PROJECT\main.py
```
`CARLA_APPLY_FPGA=1`이면 폐루프, `CARLA_LIVE_VERIFY=1`이면 시나리오 회귀 75건.
`CARLA_SENSOR_NOISE=0`이면 무잡음, `CARLA_SENSOR_NOISE_SCALE`로 진폭 배율.

---

## 5. 환경

```
Vivado 2022.2  C:\Xilinx\Vivado\2022.2\bin        (PATH에 없음)
Vitis  2022.2  C:\Xilinx\Vitis\2022.2\bin\xsct.bat
carla + pygame 는 Python 3.12 에만 설치됨
  %LOCALAPPDATA%\Programs\Python\Python312\python.exe
보드 192.168.1.10:5001, 호스트 192.168.1.20/24, MAC 00:0A:35:00:01:02
```

빌드 + 프로그래밍:
```bash
sources_1\verification\board_arrival.bat
```
1) 오프라인 게이트 2) IP 갱신 + 재빌드 + 타이밍 게이트 3) 프로그래밍 4) 브리지 확인.
**단, 빌드 단계는 함정 7번을 먼저 읽어라.** `skipbuild` 인자로 2단계를 건너뛴다.

---

## 6. RTL 파일 지도

```
sources_1/new/
  types_pkg.sv           구조체 정의. pred_data_t 의 필드 폭이 특히 중요하다
  preprocessor.sv        1차 차분, 동역학 기준값(pred_*), LUT, 마스크 1/2/3
  sensor_checker.sv      채널당 range/jump/stuck/noise/timeout
  consistency_checker.sv 관계식 하나짜리 비교기
  sensor_reliability.sv  검사기 집합 + 관계식 17개 배선 + 상태 판정(pack_ch)
  risk_types.sv          원시 위험도 tier 분류
  risk_control.sv        신뢰도 반영 유효 tier, 제동 중재, TD/MRM
  mask_20s.sv            20초 홀드 마스크
sources_1/ip/sensor_input_1_0/hdl/sensor_input_v1_0_S00_AXI.v
                         AXI 레지스터 맵 (패킹/언패킹)
```

### 상태 판정식 (확정)

```
state = (r || t || (s && c)) ? INVALID
      : (j || n || (s ^ c))  ? DEGRADED : NORMAL
```
r=range, j=jump, s=stuck, n=noise, c=consistency, t=timeout.

### 마스크 체계

```
mask_1 = |incline| >= 2990                      관계식 3/4/5 (동역학)
mask_2 = |incline| > 300 or |gyro_z| > 2000     관계식 6/7/8 (gyro)
mask_3 = |speed_x| < 100 or |incline| >= 2990   관계식 17 (조향)
mask_4 = (situation != 000)                     관계식 9~14 (정지)  <- 주행 중 항상 마스크
mask_5 = mask_4 or untrusted(accel_z)           관계식 15
mask_6 = mask_4 or untrusted(accel_y/z)         관계식 16
mask_7 = untrusted(approach_speed) or sentinel  관계식 1
mask_8 = untrusted(distance)                    관계식 2
```
`untrusted = r|t|s|j|n` (consistency 제외).

---

## 7. 반드시 알아야 할 함정 8가지

**1. 예열 20프레임을 반드시 버려라.**
보드는 시험 시작 전까지 표본을 못 받아 transport timeout이 확정된 상태다
(전 채널 INVALID). 골든 모델에는 그 사전 상태가 없다.

**2. CARLA를 물린 캡처로 비트 정확성을 판정하지 마라.**
호스트 송신 간격이 100 ms를 넘으면 PL에 실제 timeout 증거가 쌓여
디바운스 위상이 어긋난다. 경계 구간(100~120 ms)은 호스트 로그로 복원되지
않는다(임계 100 ms로 복원하면 불일치 43, 120 ms면 34, 무시하면 32).
**판정은 `--replay`(지터 없는 재생)로 한다.**
`compare_golden_vs_pl.py`가 불일치와 지터 사건의 상관을 자동 보고한다.

**3. Verilog 곱셈 결과 폭은 max(피연산자, 문맥)이다. 합이 아니다.**
좁은 LHS에 `(a*b)>>>n`을 쓰면 곱이 **먼저** wrap된다.
이 프로젝트에서 세 번 발생했다 — `pred_gyro_*_1`(16→28비트),
`pred_gyro_z_3`, `pred_accel_y_3`.
**새 pred 식을 추가할 때는 32비트 중간 신호를 먼저 만들어라.**

**4. 보드의 risk 워드는 원시 tier가 아니라 신뢰도 반영 유효 tier다.**
`classify_risk()` 결과와 직접 비교하면 신뢰도가 떨어진 구간에서 전부
불일치로 보인다. `RiskControl.risk_word()`를 써라.

**5. 신뢰도 워드는 valid 표본에서만 래치된다.**
`risk_control`의 `rel_out`이 `valid_in_rel`에서만 갱신되므로, 표본이
끊긴 **동안에는** 워드가 그대로다. 확정된 timeout은 **재개 첫 표본**에
드러난다. timeout 시험을 짤 때 관측 지점을 여기로 잡아야 한다.

**6. situation 인코딩과 sentinel.**
`000 정지 / 001 장애물등장 / 010 자세변화 / 011 날씨변화 / 100 정상`.
`mask_4 = (situation != 000)`은 "정지가 아니면 마스크"이고 올바르다.
합성 벡터에서 situation을 하드코딩하지 말고 상태에서 유도해라.
sentinel(`distance==20000 && approach_speed==0`) 마스크는 **distance에만**
걸린다. approach_speed의 0은 실제 측정값이다.

**7. 빌드 스크립트.**
- `sources_1/verification/*.bat`은 **ASCII 전용**. 한글이 OEM 코드페이지에서
  cmd 파서를 깨뜨린다.
- `program_and_bringup.tcl` / `probe_a53_state.tcl`은 XSCT 명령이라
  `xsct.bat`으로 실행해야 한다.
- **`board_arrival.bat`의 cmd 리다이렉트를 PowerShell 파이프로 감싸면**
  Vivado Tcl의 `puts`가 `can not find channel named "stdout"`으로 죽는다.
  빌드는 PowerShell에서 vivado를 직접 호출해라:
  ```powershell
  & "C:\Xilinx\Vivado\2022.2\bin\vivado.bat" -mode batch -nolog -nojournal `
    -source "sources_1\verification\build_full_project_88888mhz.tcl"
  ```

**8. 캡처와 bitstream을 짝지어라.**
구 bitstream 캡처를 신 모델로 대조하면 당연히 어긋난다. 2026-08-14에
5.396% 불일치가 이 이유로 나왔다.

---

## 8. 남은 프로젝트 계획

큰 그림은 셋이다.

**A. 정상 주행 오탐을 없앤다** (2·8번) — 현재 프로젝트의 최대 약점.
"신뢰도 기반 위험도"를 주장하려면 정상 주행에서 신뢰도가 흔들리면 안 된다.
지금은 12표본 중 1표본꼴로 흔들리고, 그게 가속 상한까지 건드린다.

**B. 검출 못 하는 구멍을 막는다** (1번) — 안전 관점 최우선.
정지 표적을 향한 고착 레이더가 원리적으로 검출 불가다.

**C. 검증을 마무리한다** (3·5·6·7·9·10번) — 미검증 항목 정리와 사양 대조.

권장 순서: **1 → 2 → 8 → 10 → 3 → 나머지.**
1번은 안전, 2·8번은 프로젝트 주장의 근거, 10번은 나머지 검증의 전제다.

작업 단위마다 반드시:
`run_offline_verification.bat` → `OVERALL: PASS` 확인 → 보드 재확인
(`--replay` 또는 `--frames`) → 수치와 로그 경로로 보고.

---

## 8-1. 열린 항목 (우선순위 순)

### 1. stuck 교차 게이팅 사각지대 — 안전 관점 최우선

거리 stuck은 `|approach_speed| >= 20`(0.2 m/s)일 때만 testable이고,
접근속도 stuck은 거리 채널이 정상일 때만 검사된다
(`stuck_mask_1s_or_20s[CH_APSP]`가 distance 고장을 OR한다).

**정지 표적을 향한 고착 레이더는 원리적으로 검출되지 않는다.**
실측: `distance:stuck` 주입 시 접근속도가 8 raw(0.08 m/s)라 testable이
아니었고, `approach_speed:stuck` 주입 시 거리 진동이 distance를 DEGRADED로
만들어 마스크됐다. 라이브 시나리오에서 이 2건이 실패하며, **오늘 변경
이전에도 간헐적으로 실패했다**(회귀 아님).

교차 게이팅 자체는 타당하다. 문제는 **검사가 꺼졌다는 사실이 밖으로
드러나지 않는 것**이다. "검사 불가" 상태를 별도 표시하거나 레이더 자체
헬스 신호를 쓰는 방안이 필요하다. **미승인.**

### 2. NOISE_THRESHOLD_2 (부호 반전 7회) — 남은 오탐의 주범, 근거 확보됨

수정 후 남은 오탐은 consistency가 아니라 **noise 검사기**가 지배하고,
그중에서도 `NOISE_THRESHOLD_1`(크기)이 아니라
**`NOISE_THRESHOLD_2` = 10표본 창 안의 부호 반전 7회 초과**가 원인이다.

`NOISE_THRESHOLD_1`은 accel 기준 500이라 10창 합 5000(50 m/s²)이어야 걸린다.
현실적으로 도달하지 않는다. 반면 CARLA 가속도계는 평균 주변에서 진동해
부호가 자주 뒤집힌다. `delta_accel_y`를 직접 센 결과:

| 잡음 배율 | 10창 반전 >7 비율 | accel_y 오탐률 |
|---|---|---|
| OFF | 11.5% | 11.50% |
| x1 | 7.8% | 7.76% |
| x3 | 2.9% | 2.87% |

**수치가 그대로 일치한다.** 임계값 재검토 대상이다. **미승인.**

부수 사실: 잡음 모델이 **저주파**라 더할수록 delta가 매끄러워져 반전이
오히려 줄어든다. 그래서 **현재 잡음 모델로는 이 검사기를 시험할 수 없다.**
강건성 시험을 제대로 하려면 백색 성분을 섞어야 한다.

lux stuck은 바닥 진동 도입으로 해결됐다(6.4% → 0.00%).

### 3. yaw ±180° 경계 미처리

관계식 8 잔차 최대 128,661,992 = `36000 × C_GYR(3574)`.
`incline_z`가 ±180°를 넘는 순간 차분이 36000이 되어 순간적으로 깨진다.
개루프 캡처에서 2회 관측. `delta_incline_z`를 ±18000 기준으로 되감으면
해결된다. **미승인.**

### 4. 충돌 시나리오 입력이 모순이다

`world_scenario_controller.py`가 접근속도 10 m/s를 주장하면서 거리를
고정한다(tier 1=35 m 등). PL이 stuck과 관계식 1로 정확히 잡아내
신뢰도를 떨어뜨리고, 그 결과 충돌 tier가 상향되어 시험이 실패한다.
**PL은 옳고 시나리오가 틀렸다.**

주의: TTC 밴드 폭은 tier 3이 0.5초, tier 1/2가 1초, tier 4가 1.5초다.
거리를 접근속도에 맞춰 줄이면 케이스 지속시간이 밴드 폭보다 짧아야 한다.
근본적으로는 **실제 표적을 띄우고 레이더가 측정하게** 하는 것이 옳다.
**미승인.**

### 5. 관계식 15 실보드 확인

`pred_accel_y_3`의 12비트 곱 wrap을 고쳤으나 실보드 확인이 없다.
`mask_5`가 주행 중 항상 마스크하므로 **경사면 정차** 자극이 필요하다.
오프라인 `standstill_slope` 시나리오로는 통과 확인됨.

### 6. 사양 문서 대조

구글 독스(`신뢰도 로직(claude) 표1`, `(최신)consistency check(claude)`)와
관계식 17개 수식·임계 17개·스케일 상수·디바운스 U/D/N·마스크 조건을
대조해야 한다. 사람이 해야 하는 일이다.

### 7. 미검증으로 남은 것

- 관계식 1: 실보드에서 물리적으로 타당한 접근 자극으로 돌아본 적 없다
  (오프라인 `brake_ice`로는 검증됨)
- 관계식 2: active로 집계되나 무표적이라 0 vs 0을 비교한 것뿐이다
- 폐루프에서 FPGA 명령이 실제로 차량에 적용됐는지는 로그 부재로만 확인

### 8. 관계식 3/7/8의 기준값 정확도 — 오탐의 나머지 절반

관계식 3(accel_x)은 속도를 2표본 차분해 가속도를 추정한다. 급가감속 구간에서
실제 가속도와 벌어져 잔차 최대 134(임계 76), 확정률 4.6%다.
관계식 7(gyro_y) 2.6%, 관계식 8(gyro_z) 1.9%도 같은 성격이다.

관계식 4는 구심항을 넣어 0.00%가 됐다. **같은 방식으로 나머지도 물리를
보강할 수 있는지** 보는 것이 정공법이다. 임계값을 올리는 것은 검출 민감도를
깎으므로 최후 수단이다. **미승인.**

### 9. lux 오탐률 불일치 (경미)

배율 x1에서 보드 캡처는 6.38%, 모델 통제실험은 0.00%다. 주행이 달라 조도
조건이 달랐을 가능성이 크나 확인되지 않았다. 같은 궤적으로 보드 재생
대조를 돌리면 바로 갈린다.

### 10. 강건성 스윕의 검출 측 절반

배율별로 고장을 주입해 검출률·검출 지연이 유지되는지 봐야 한다.
`CARLA_LIVE_VERIFY=1` 주행 후 `fault_latency_metrics.py`.
배율마다 약 10분. **오탐 곡선만으로는 강건성을 주장할 수 없다.**

### 11. 잡음 모델에 백색 성분 추가

현재 모델은 저주파라 `NOISE_THRESHOLD_2`(부호 반전)를 전혀 스트레스하지
못하고 오히려 완화한다. 부호 반전 판정을 시험하려면 표본 단위로 부호가
바뀌는 성분이 필요하다. 2번 항목의 판단이 선 뒤에 하는 것이 순서다.

---

## 9. 하지 말아야 할 것

- 기존 위험도/신뢰도 알고리즘을 임의로 바꾸지 마라. 파라미터 변경도 근거를
  제시하고 승인을 받아라.
- **불일치가 났다고 임계값부터 올리지 마라.** 2026-08-14의 세 결함은 전부
  임계값이 아니라 기준값 정확도 문제였고, 임계값을 그대로 둔 채 해결됐다.
  `TH_ACC_STOP=4`도 "양자화 바닥일 것"이라 의심했다가 측정해 보니
  잔차 최대 1로 충분했다. **의심은 반드시 측정으로 확인해라.**
- 검증 실패를 숨기거나 시험 조건을 바꿔 PASS를 만들지 마라.
- 실행하지 않은 검증을 PASS라고 쓰지 마라. 미검증은 미검증이라고 써라.
- GUI(pygame 창)를 손으로 조작하는 검증은 불가능하다.
  `live_scenario_verifier.py`(`CARLA_LIVE_VERIFY=1`)로 프로그램 구동해라.

---

## 10. 최근 작업 기록

전체 근거와 수치는
`verification_reports/golden_model_closure_and_rtl_defects_20260814.md`에 있다.

**2026-08-14 1차** — 골든 모델 완성(관계식 17건 + 마스크 8종), RTL 결함 3건
수정(`pred_gyro_z_3`/`pred_accel_y_3` 곱셈 폭, 관계식 4 구심항 누락),
인터페이스 해상도 복원(속도 8→14비트, 조향 5→8비트, reg8 여유 비트 사용).
오탐률 accel_x 70.1%→9.6%, accel_y 55.3%→7.1%, gyro_z 27.7%→2.9%.
**임계값은 하나도 바꾸지 않았다.**

**2026-08-14 2차** — 상태 판정식 확정, TD/MRM + 유효 tier 골든 모델 구현
(비교 범위에 risk 워드/HUD/TD/MRM 추가), sample_seq 보류 방식 timeout 시험
확보, `standstill_slope` 시나리오로 관계식 9~16 최초 활성화,
`CARLA_SENSOR_NOISE_SCALE` 추가.

**2026-08-14 3차** — 환경 채널 바닥 진동 도입(온습도·조도 오탐 100/100/93%
→ 0%), 통제 실험 방식 확립(`noise_injection_sweep.py`), 강건성 곡선 산출
(한계 x2.0, INVALID은 x4까지 0건), 남은 오탐의 주범이
`NOISE_THRESHOLD_2`(부호 반전)임을 수치 일치로 규명.
`test_scenario_pl_alignment` 에 바닥 진동 성질 시험 신규 추가(26건).

**미커밋 상태다.** RTL 3개 + AXI IP 1개 + Python 다수 + 문서.
