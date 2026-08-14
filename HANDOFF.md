# HANDOFF — FPGA 신뢰도/위험도 기반 자율주행 로직

> **먼저 이것부터 (2026-08-15)**
>
> 1. **골든 모델(`pl_model.py`)이 정답 코드다.** 불일치가 나면 RTL 을 고친다.
>    (한때 폐기했다가 뒤집혔다. 3절 참조)
> 2. **골든과 RTL 이 지금 의도적으로 다르다.** 신뢰도 판정식을 골든에만
>    반영했다. **4-1 절의 RTL 반영이 다음 최우선 작업**이고, 그래야 CARLA 에서
>    stuck 상태가 안 흔들리고 MRM 이 발동한다.
> 3. **실행 전 좀비 python 을 정리하라.** UDP 5002 가 잡혀 있으면 보드 응답 0
>    인 캡처가 조용히 만들어진다 (7절 함정 9).
> 4. 승인은 채팅이 아니라 `AskUserQuestion` 선택 창으로 받는다.

이 문서 하나만 읽고 바로 이어서 작업할 수 있도록 썼다.
작업 폴더는 `C:\Users\jiho0\GitProjects\FPGA-Autonomous-Vehicle-Project`.
한국어로 답하고, RTL 수준 용어는 설명 없이 써도 된다.

작업 규칙은 `CLAUDE.md`에 있다. **먼저 읽어라.** 특히 5절(Golden Model 규칙)과
7절(검증 결과 보고 형식)은 이 프로젝트의 판단 기준 자체다.
승인은 반드시 `AskUserQuestion` 선택 창으로 받는다(2절).

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

### 구현 검증 — 전송 계층 확정, 판정은 갱신 중

전송(UDP·AXI·sequence)은 확정이다.  지터 없는 재생에서 전 프레임 응답,
sequence 불일치 0, 왕복 지연 평균 0.7 ms 미만.

**신뢰도/제어 판정은 2026-08-15 에 골든을 크게 바꿨고 RTL 은 아직 안 바꿨다.**
따라서 지금 골든과 RTL 은 **의도적으로 다르다**(4-1 절 참조).

### 검증 커버리지

캡처 20260814_212652 (3,452 표본) 기준, 보드 vs 골든.
**단 이 수치는 판정식을 바꾸기 전 골든 기준이다.**  RTL 반영 후 재측정해야 한다.

| 항목 | 결과 |
|---|---|
| 신뢰도 11채널 상태 | 불일치 7 / 37,972 (0.018%) — **전부 지터 구간** |
| HUD 경고 / MRM / TD 잔여초 | 각 **100.00%** |
| risk 워드(유효 tier) | 3452건 중 불일치 4 (99.88%) |
| **최종 가속 / 제동 / 제한속도** | 각 3452건 중 불일치 4 (99.88%) |
| **최종 조향 / 전조등 / 비상등** | 각 **100.00%** |
| 지터 없는 재생 | 3,472 프레임 전송 정상, RTT 평균 0.68 ms |
| transport timeout | 결정론적 시험 PASS (11/11 INVALID) |
| 관계식 1~17 | 구현됨. **사양 문서 대조는 안 됨** (8절 C) |
| 라이브 시나리오 | 71 PASS / 4 FAIL (실패 4건 원인 규명, RTL 무관) |

불일치 4~7건은 전부 seq 1566~1649 의 **호스트 지터 구간**이며 로직 차이가
아니다.  판정은 `board_smoke_test.py --replay` 로 하라.

### 빌드 (현재 보드에 올라간 것)

```
setup WNS +0.068 ns   TNS 0   hold WHS +0.011 ns   THS 0
DRC error 0 / critical warning 0
LUT 18.68% FF 6.75% BRAM 0%
```

**여유가 얇다(+0.068 ns).** noise `&&` 변경 + gyro 비교항 신설로
+0.311 → +0.068 ns까지 줄었다. 로직을 더 얹으면 음수가 된다.

### 정상 주행 오탐률 — 2026-08-14 실측으로 갱신됨

캡처 `pl_capture_20260814_212652.csv` (3,472표본, 개루프, 고장 미주입, 잡음 ON)

| 채널 | 이전 개루프 | **현재** | 비고 |
|---|---|---|---|
| accel_x | 9.57% | **4.87%** | 잔존. 전부 관계식 3 |
| accel_y | 7.14% | **0.00%** | 해결 |
| accel_z | — | **0.12%** | 사실상 해결 |
| gyro_x | 0.44% | **0.00%** | 해결 |
| gyro_y | 2.63% | **0.00%** | 해결 |
| gyro_z | 2.87% | **0.00%** | 해결 |
| lux | 6.39% | **0.00%** | 해결 |
| 나머지 4채널 | 0~2.8% | **0.00%** | |

**INVALID 오탐 11채널 전부 0건.**
`noise` 검사기 확정 **0%** — `||` → `&&` 변경이 실주행에서 확인됐다.
lux 클램프 수정은 새 주행이라야 검증 가능했고, 이번에 확인됐다.

**남은 것은 관계식 3(accel_x) 하나다.** 아래 "남은 최대 결함"의 2번(noise
부호 반전)은 해결됐고, 8번(관계식 3)만 남았다.

상세: `verification_reports/보드_실주행_검증결과_20260814.md`

### 보드 대조 (2026-08-14)

| 경로 | 결과 |
|---|---|
| CARLA 물린 캡처 | 37,972건 중 불일치 7 (0.018%) — **전부 지터 구간** |
| **지터 없는 재생 3,472프레임** | **전수 일치 PASS**, RTT 평균 0.68 ms |
| HUD / MRM / TD 잔여초 | **각 100.00% 일치** |

지터: 평균 51.84 ms, p95 53.42, 최대 269.13, **100 ms 초과 63회(1.81%)**.
원인은 호스트 GPU 부하(**AMD Radeon 840M 내장, 4 GB 공유**)다. RTL/보드
문제가 아니다. CARLA를 `-quality-level=Low -ResX=640 -ResY=480`으로 띄워라.

### 화면 (건드리지 마라)

`main.py:560`의 `smoothscale`은 **종횡비를 보존하지 않는다.** 창(1280×720,
16:9)과 주 모니터(1536×960, **16:10**)의 비율이 다르므로 **F11 전체화면이나
창 리사이즈를 하면 세로로 약 11% 늘어난다.** 카메라는 세션 시작 시 한 번만
생성되고 리사이즈 때 재생성되지 않는다.

사용자가 **종횡비 보존 처리를 넣지 않기로 결정했다.**
→ **F11과 창 크기 조절을 쓰지 마라.** 기본 실행 경로는 비율이 정상이다.

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
| timeout 시험 | **O** | 보드 11/11 INVALID |
| **위 통제 실험 표 (OFF~x4)** | **X** | **폐기된 참조 모델 계산이다. 재측정 필요** |
| 스윕 캡처의 지터 없는 재생 대조 | **X** | 미실행 |

**통제 실험 표는 보드 측정이 아니다.** 같은 궤적에 잡음만 바꾸려면 CARLA를
다시 돌릴 수 없어 오프라인 재주입을 썼고, 판정은 폐기된 참조 모델이 했다.
**이 표 전체를 신뢰하지 마라.** 필요하면 보드로 재측정해야 한다.

부분 교차 검증은 됐다. 배율마다 따로 주행한 캡처(궤적은 다름)와 비교하면

| 항목 | 보드 캡처 | 폐기된 모델 계산 |
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

## 3. 검증 체계 — 골든 모델이 정답이다

### 2026-08-15 방향 전환 (중요)

한때 `pl_model.py` 를 "RTL 에서 파생돼 사양 오해를 못 잡는다"는 이유로
폐기했었다.  **그 결정은 뒤집혔다.**  사용자가 다음과 같이 정했다.

> "golden은 정답 코드여야 한다. 이에 맞게 rtl을 수정할 의향이 있다."

즉 **골든이 사양의 구현체이고, RTL 이 거기에 맞춰야 한다.**
불일치가 나오면 이제 RTL 을 고친다.  골든을 RTL 에 맞추지 마라.

다만 골든의 출신은 정직하게 알고 있어야 한다.  최초 구현은 RTL 에서
읽어 옮긴 것이고, 사양 문서(구글 독스 표1 / (최신)consistency check)와는
**아직 대조된 적이 없다.**  문서를 확보하면 그것과 먼저 맞춰야 한다.

### 두 질문을 섞지 마라

| 질문 | 판정 수단 |
|---|---|
| A. FPGA 가 골든대로 동작하는가 | `compare_golden_vs_pl.py` (시뮬/보드 대조) |
| B. 골든(=알고리즘)이 옳은가 | 물리·산술 검토, 고장 주입 라벨, 안전 불변조건, 사양 문서 |

A 의 불일치 원인은 셋 중 하나이며 반드시 구분한다.

1. RTL 이 골든과 다르게 구현됨 -> **RTL 을 고친다**
2. 합성/타이밍/AXI 전송 문제
3. 골든이 사양을 잘못 옮김

구분법: 같은 벡터를 xsim 에 재생한다.  시뮬은 맞는데 보드만 틀리면 2번.

**호스트 지터가 100 ms 를 넘으면 PL 에 실제 timeout 증거가 쌓여 디바운스
위상이 어긋난다.**  판정은 반드시 `board_smoke_test.py --replay`(지터 없음)
로 하라.  CARLA 를 물린 캡처의 불일치는 지터 상관을 먼저 확인한다.

## 4. 도구

전부 `CARLA_FPGA_PROJECT/` 안에 있고 표준 라이브러리만 쓴다(carla 제외).

| 파일 | 하는 일 |
|---|---|
| `pl_model.py` | **골든 모델(정답).** 관계식 1~17, 검사기 5종, 유효 tier, TD/MRM, 제어 중재 |
| `compare_golden_vs_pl.py` | 골든 vs 시뮬/보드 대조. 신뢰도·risk·**제어**·지터 상관 |
| `td_mrm_timing_test.py` | TD 카운트다운/MRM 발동 시각을 보드에서 직접 측정 |
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
보드 UDP 왕복 시험. 전송 계층만 판정한다.

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

## 4-1. 골든 ↔ RTL 차이 (2026-08-15 현재, 최우선 작업)

**골든에만 있고 RTL 에는 없다.**  CARLA 에서 보이는 것은 아직 옛 RTL 동작이다.
RTL 에 반영하는 것이 지금 가장 중요한 다음 작업이다.

### (1) 상태 판정식 교체

```
# 골든 (pl_model.py _resolve_states)
c_independent = c and not (r or j or s or n or t)
soft_hard     = jump_hard or noise_hard          # 지속 격상, 배수 3
INVALID  = r or t or soft_hard
DEGRADED = j or n or s or c_independent
```

```systemverilog
// RTL (sensor_reliability.sv pack_ch) — 아직 옛 식
state = (r || t || (s && c)) ? INVALID :
        (j || n || (s ^ c))  ? DEGRADED : NORMAL;
```

**왜 바꿨나.** `(s && c)` 는 "stuck 과 consistency 는 독립적 증거" 를 전제로
하는데 그 전제가 틀렸다.  값이 얼어붙으면 기준값만 계속 움직이므로
consistency 는 **논리적 필연으로** 따라 확정된다.  같은 고장을 두 번 센다.
그 결과 RTL 에서는 `(s^c)` 와 `(s&&c)` 사이를 오가며 상태가
DEGRADED ↔ INVALID 로 흔들리고, `td_condition` 이 끊겨 **TD 카운트가 계속
초기화되어 MRM 이 영영 발동하지 않는다.**

반대로 다른 넷이 깨끗한데 consistency 만 서면 그것은 넷이 구조적으로 못 보는
고장(바이어스/스케일/드리프트/축 뒤바뀜)이다.  실측 근거: 캡처
20260814_212652 에서 accel_x 는 range/jump/stuck/noise 가 전부 0.00% 인데
관계식 3 만 4.9% 확정됐다.  그래서 consistency 를 버리지 않고,
**다른 검사가 전부 깨끗할 때만 증거로 센다.**

`stuck` 은 DEGRADED 다(사용자 결정).  한때 INVALID 로 올렸다가 환경 채널이
바로 INVALID 로 튀어 되돌렸다.  **stuck 만으로는 TD/MRM 이 발동하지 않는다.**
MRM 시험은 range 나 timeout 으로 하라.

**지속 격상은 jump/noise 에만 건다.**  consistency 에도 걸었더니 관계식 3 의
만성 오탐 4.9% 가 그대로 INVALID 오탐 135건으로 증폭됐다(불변조건 파괴).
카운터 포화 상한을 올리면 복구가 느려져 오탐률이 4.87% -> 5.48% 로 나빠지는
것도 실측했다.  **격상은 그 검사의 오탐이 0 일 때만 안전하다.**

### (2) 제어 중재 (골든에 새로 이식)

`risk_control.sv` 305-655행을 `RiskControl.arbitrate()` 로 옮겼다.
충돌 4단계, 노면 A/B 3단계, 시야 A/B, 자세 A/B/C, 가속 8소스 min,
제동 마찰 블렌딩, 조향 최근접 선택, 제한속도 min, MRM 오버라이드.
`spd_limit_*` 의 정수 곱/시프트(`* 922 >> 10` 등)까지 동일하다.

RTL 은 원래 잘 동작하므로 **RTL 을 정답으로 두고 옮긴 것**이다.
이 부분은 RTL 수정이 필요 없다.

---

## 4-2. 2026-08-15 에 고친 것 (골든/Python)

| 대상 | 내용 |
|---|---|
| `pl_model.py` | 상태 판정식 교체, `stuck_hard`/`jump_hard`/`noise_hard` 지속 카운터, `arbitrate()` 제어 중재, `step()` 이 시간 자동 진행 |
| `compare_golden_vs_pl.py` | **제어 대조 추가**(REG13/14), `decode_input_words` 에 제어 입력 5필드 추가, `--skip` 예열 |
| `control_panel.py` | distance 계열 주입 시 기준 거리 **고정** |
| `main.py` | timeout 주입을 25프레임마다 1표본 통과로, `CARLA_INJECT_FAULT` / `CARLA_INJECT_RISK` 무인 주입 |
| `vehicle_controller.py` | **속도 기준 자동 다운시프트** (대역 20/55/80, 이력 3 km/h), `force_downshift` 1회성 경로 |
| `vehicle_command.py` | `force_downshift` 필드 (MRM 1회성 다운시프트) |
| `CLAUDE.md` | 승인은 `AskUserQuestion` 선택 창으로 받는다 |

### 왜 그렇게 고쳤는지 — 핵심 3가지

**timeout 이 화면에 NORMAL 로 뜨던 이유.**  `risk_control.sv` 는
`if (valid_in_rel) rel_out <= rel_in;` 이라 **valid 표본에서만** 신뢰도 워드를
래치한다.  `sample_seq` 를 완전히 고정하면 PL 이 timeout 을 확정해도 바깥으로
내보내지 않아 마지막 값(NORMAL)이 남는다.  그래서 25프레임마다 1표본을
통과시킨다.  RTL 의 `timeout_confirm_hold(DROP_N=2)` 가 확정 timeout 을 복구
첫 표본까지 유지하도록 만들어져 있어 그 표본에서 11채널 INVALID 가 래치된다.

**stuck 이 오르내리던 이유.**  주입기가 매 프레임 `min(현재거리, 80.0)` 을
재계산했다.  레이더가 표적을 놓치면 sentinel(20000)로 돌아가고 다음 프레임에
다시 8000 으로 끌려온다.  20000->8000 은 jump, 8000->20000 은 sentinel 마스크로
전 진단 무효화.  **stuck 이 STUCK_N=10 직전에 계속 리셋되어 확정 0회**였다
(캡처 20260814_234027 실측, 상태 전이 54회).  첫 프레임 값을 고정하도록
바꾼 뒤 실주행 41초 연속 DEGRADED 유지를 확인했다(캡처 20260815_000656).

**MRM 이 골든에서 안 뜨던 이유.**  `tick_second()` 를 부르는 곳이
`compare_golden_vs_pl` 하나뿐이라 나머지 도구에서는 시간이 흐르지 않아
`td_remain_sec` 가 영원히 11 이었다.  `step()` 이 기본으로 표본 주기(50 ms)를
흘리도록 바꿨다.  TD/MRM 로직 자체는 사양대로 이미 구현돼 있었다.

### MRM 사양 (UNECE R157) 과 구현 상태

```
td_locked : INVALID 로부터 5초간 manual 전환이 없으면 ON
            ON 이면 DEGRADED/NORMAL 로 돌아가도 카운트 계속
            OFF 면 상태 복귀 시 카운트 중지·초기화
카운트 0  -> MRM 발동
MRM 중 manual_mode -> 수동 주행, TD 알림 꺼짐
```

**①②③ 을 동시에 수행한다.**  그 결과 차량이 완전 정차하면 ④ 정차를
유지하고, ⑤ 수동 개입이 있으면 manual mode 로 주행을 시작한다.

| R157 절차 | 구현 | 비고 |
|---|---|---|
| ① Hazard ON | `final_hazard = True` | |
| ② Brake level 3 | `final_brake = 3` | 정차까지 계속 유지 |
| ③ 기어 1단 내리기 | **1회성** | 아래 참조 |
| ④ 완전 정차 / 정차 유지 | brake 3 유지로 따라온다 | 별도 상태 없음 |
| ⑤ 수동 개입 시 Manual | `manual_mode` 가 TD 초기화 -> MRM=0, 알림 OFF | |

**③ 은 반드시 1회성이다 (2026-08-15 사양 확정).**  MRM 이 서는 순간 한 단만
내리고, **그 뒤의 변속은 Python 자율주행 로직(속도에 맞는 기어 단수)이
이어받는다.**  매 표본 `gear-1` 을 내보내면 "계속 내리라" 는 뜻이 되어
사양과 다르다.  변속 후 0.5초 텀(`SHIFT_DELAY`)은 그대로 지킨다.

구현 위치가 둘로 나뉜다.
- 골든 `RiskControl.arbitrate()` : `mrm_downshift_done` 플래그로 1회만 적용.
  rpm 조건은 걸지 않는다(정차 과정에서 rpm 조건을 기다리면 아예 안 내려간다).
- Python `main.py` : `fpga_result.mrm` **상승 엣지**에 `command.force_downshift`
  를 한 번 세운다.  (PL 의 gear 출력은 적용하지 않는 설계이므로 여기서 건다.)
- Python `vehicle_controller.process_gear()` : `pending_mrm_downshift` 로
  **래치**한 뒤 RPM 조건 없이 한 단 내린다.

**래치가 필요한 이유.**  main.py 는 한 프레임만 플래그를 세우는데, 그 프레임이
마침 `SHIFT_DELAY`(0.5초) 안이면 요청이 그대로 사라져 MRM 다운시프트가
일어나지 않는다.  래치하면 수행 가능해질 때까지 유지된다.

**0.5초 텀은 MRM 다운시프트 시점부터 센다.**  다운시프트 시
`last_shift_time` 을 갱신하므로 자율주행 로직의 다음 변속은 그 시점 기준
0.5초 뒤에야 가능하다.  실측:

```
t=0.00s gear=3 pending=True   MRM 발동, 직전 변속 0.5초 안이라 보류
t=0.55s gear=2 pending=False  MRM 다운시프트 수행 (n -> n-1)
t=0.55s gear=2                0.5초 안이라 추가 변속 없음
t=1.10s gear=1                자율주행 로직 변속 재개
```

골든 실측: `11.00s MRM=1 gear 3->2`, `11.05s gear=3` (입력 기어로 복귀 = Python 제어).

골든 실측: `11.00초 MRM=1, brake=3, gear 3->2, hazard=1`,
manual 개입 후 `잔여=11 MRM=0 TD알림=0`.

### 기어는 PL 이 아니라 Python 이 정한다

사용자 결정: **FPGA 제어 영역 밖은 CARLA 쪽 로직 그대로 쓴다.**
`main.py` 는 `fpga_result.gear` 를 적용하지 않는다(의도된 것이다).
문제는 Python 에 **속도 기준 다운시프트가 없어서** 감속해도 기어가 안
내려간 것이었다.  `GEAR_RATIO` 주석의 대역(20/55/80 km/h)을 상수로 올리고
3 km/h 이력을 붙였다.

**RPM 기준으로 내리면 안 된다.**  `DOWNSHIFT_RPM=3000` 은 2단에서 25.4 km/h
인데 1->2 업시프트는 20.0 km/h 라 그 사이에서 0.5초마다 헌팅한다.

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

## 7. 반드시 알아야 할 함정

**1. 예열 20프레임을 반드시 버려라.**
보드는 시험 시작 전까지 표본을 못 받아 transport timeout이 확정된 상태다
(전 채널 INVALID). 비교 대상에는 그 사전 상태가 없다.

**2. CARLA를 물린 캡처로 비트 정확성을 판정하지 마라.**
호스트 송신 간격이 100 ms를 넘으면 PL에 실제 timeout 증거가 쌓여
디바운스 위상이 어긋난다. 경계 구간(100~120 ms)은 호스트 로그로 복원되지
않는다(임계 100 ms로 복원하면 불일치 43, 120 ms면 34, 무시하면 32).
**판정은 `--replay`(지터 없는 재생)로 한다.**
지터가 100 ms를 넘으면 PL에 실제 timeout 증거가 쌓여 디바운스 위상이
어긋난다. 판정은 반드시 `board_smoke_test.py --replay`(지터 없음)로 하라.

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


### 9. 실행 전 좀비 python 을 반드시 정리하라

UDP 5002 를 앞선 실행이 잡고 있으면 `WinError 10048` 로 FPGA 초기화가
실패하고 **보드 응답이 0 인 캡처가 조용히 만들어진다**(로그를 안 보면 모른다).

```powershell
Get-NetUDPEndpoint -LocalPort 5002 -ErrorAction SilentlyContinue
Get-Process -Name "python*" | Stop-Process -Force
```

캡처를 분석하기 전에 `fpga_response_valid` 가 1 인지 먼저 확인하라.

### 10. 무인 실행 중 CARLA 창을 클릭하지 마라

제어판 클릭 핸들러가 살아 있어 주입이 초기화되거나 다른 고장이 켜진다
(실측: `[INJECT] all injections and scenarios cleared` -> `distance/range: ON`).
`CARLA_INJECT_FAULT` 로 주입했더라도 클릭 한 번에 날아간다.

### 11. 지속 격상은 오탐이 0 인 검사에만 걸어라

만성 오탐이 있는 검사에 지속 격상을 걸면 DEGRADED 오탐이 그대로 INVALID
오탐으로 증폭된다.  실측: consistency 에 걸었더니 accel_x 가
4.87%(INVALID 0) -> 5.48%(INVALID 135) 가 됐다.
카운터 포화 상한을 올리는 것만으로도 복구가 느려져 오탐률이 오른다.

### 12. 골든에 없던 것을 넣으면 새 결함이 드러난다

제어를 골든에 이식하자마자 `decode_input_words` 에 `accelerator`/`brake`/
`gear`/`rpm`/`speed_limit` 이 **아예 없다**는 것이 드러났다(전부 0 으로 읽혀
정상 상황에서도 `accel=0`).  비교 대상에 없던 신호는 검증된 적이 없다는
뜻이다.  "전수 일치" 를 인용할 때 **무엇이 비교 대상이었는지** 반드시 확인하라.

## 8. 다음 작업 (2026-08-15 기준 우선순위)

### A. RTL 을 골든에 맞춘다 — 최우선

4-1 절의 상태 판정식을 `sensor_reliability.sv` / `sensor_checker.sv` 에
반영한다.  이것을 해야 CARLA 에서 stuck 상태가 안 흔들리고 MRM 이 뜬다.

```systemverilog
// sensor_reliability.sv pack_ch
c_independent = c && !(r || j || s || n || t);
state = (r || t || soft_hard) ? INVALID :
        (j || n || s || c_independent) ? DEGRADED : NORMAL;

// sensor_checker.sv : jump/noise 지속 카운터 (포화 상한 3배) -> *_hard 출력
```

**주의: setup 여유가 +0.068 ns 다.**  판정식이 복잡해지고 카운터 비트가
늘어 음수가 될 수 있다.  빌드 게이트가 실패시키므로 깨지면 즉시 안다.
음수가 되면 `pred_accel_y` 경로의 DSP48 파이프라인이 근본 대책이지만
latency 가 바뀌어 valid 정렬 수정이 함께 필요하다.

반영 후 순서:
1. `run_offline_verification.bat` -> OVERALL: PASS
2. 재빌드 (타이밍 게이트 통과 확인)
3. `program_and_bringup.tcl` 로 보드에 올림
4. `board_smoke_test.py --replay <capture>` 로 골든 대조 -> 차이 0 확인
5. CARLA 에서 stuck/range/timeout 버튼으로 실동작 확인

### A-2. MRM 이 INVALID 즉시 발동한다 — 사용자 재확인 (미해결)

**증상**: 신뢰도가 INVALID 되는 순간 MRM 이 실행된다.  사양은 INVALID 로
시작한 10초 카운트가 0 에 도달했을 때다.

**골든에서는 사양대로다.**  실측(range 고장 주입):

```
 0.05s 잔여=11 MRM=0     11.00s 잔여=0 MRM=1 brake=3 gear 3->2 hazard=1
```

따라서 보드(RTL) 쪽 문제로 보이며, 가장 유력한 원인은 **`td_locked` 가
직전 시험에서 래치된 채 남아 있는 것**이다.

```systemverilog
// risk_control.sv — td_locked 는 reset 과 manual_mode 로만 풀린다
if (td_invalid_duration >= 3'd4) td_locked <= 1'b1;
...
if (td_condition || td_locked) begin ... end     // locked 면 계속 카운트
else begin if (!td_locked) td_remain_sec <= 4'd11; end   // locked 면 0 유지
```

한 번 MRM 까지 간 뒤 고장을 해제해도 `td_locked=1` 이라 `td_remain_sec` 가
**0 에 머문다 -> MRM 이 계속 켜져 있다.**  그 상태에서 다음 고장을 주입하면
"INVALID 되자마자 MRM" 으로 보인다.

**확인 방법**: 고장 주입 전에 `fpga_td_remain_sec` 를 읽어라.
11 이 아니면 이미 래치된 것이다.  `main.py` 는 시작 20표본 동안
`fpga_manual_mode` 를 세워 초기화하지만(`sample_seq < pl_startup_reset_samples`),
**세션 중간에 래치된 것은 M 키로 수동 전환하기 전까지 안 풀린다.**

**판단이 필요한 지점**: 사양상 "MRM 은 운전자 개입 전까지 유지" 가 맞는지,
아니면 고장이 해소되면 풀려야 하는지.  사용자 확인 후 RTL/골든 양쪽에
반영해야 한다.  현재 골든은 RTL 과 같은 래치 동작이다.

---

### B. CARLA 실주행 미검증 항목

- 기어 속도 기준 다운시프트 (MRM 감속 시 기어가 따라 내려가는지)
- timeout 주입이 화면에 INVALID 로 뜨는지
- MRM 이 실제로 차를 세우는지 (RTL 반영 후에야 가능)

### C. 사양 문서 대조

`sensor_reliability.sv` 주석의 구글 독스(신뢰도 로직 표1 / (최신)
consistency check)를 확보해 골든과 대조한다.  **골든이 정답이 되려면
이 단계가 필요하다.**  현재 골든은 RTL 에서 파생된 것이고 문서와 맞춰본
적이 없다.

### D. 관계식 3 (accel_x 오탐 4.87%)

정상 주행 오탐의 유일한 잔존 항목.  속도 2표본 차분으로 가속도를 추정하는
방식의 한계다.  관계식 4 가 구심항으로, 6/7/8 이 물리 보강으로 0% 가 된 것과
같은 방식이 가능한지 본다.  **미승인.**

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

### 2. ~~NOISE_THRESHOLD_2~~ — **해결됨 (2026-08-14)**

`||` -> `&&` 로 바꿔 실주행 확정률 0.00%. 아래는 당시 기록이다.

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

### 3. ~~yaw ±180° 경계~~ — **해결됨.** preprocessor 에서 최단 경로로 되감는다.

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

### 8. 관계식 3 — **정상 주행 오탐의 유일한 잔존 항목 (미승인)**

**2026-08-14 갱신.** 관계식 7/8은 gyro 비교항 수정(중점 보정 + roll/pitch
부호 반전)으로 **확정률 0.0%가 됐다.** 남은 것은 관계식 3 하나다.

관계식 3(accel_x)은 속도를 2표본 차분해 가속도를 추정한다. 급가감속 구간에서
실제 가속도와 벌어진다.

| 캡처 | 원시위반 | 확정 | 잔차 p90 | 잔차 최대 (임계 76) |
|---|---|---|---|---|
| 20260814_212652 | 6.0% | **4.9%** | 42 | 131 |

관계식 4는 구심항을 넣어 0.00%가 됐고, 6/7/8도 물리 보강으로 0.0%가 됐다.
**같은 방식으로 관계식 3도 물리를 보강할 수 있는지** 보는 것이 정공법이다.
임계값을 올리는 것은 검출 민감도를 깎으므로 최후 수단이다. **미승인.**

### 9. lux 오탐률 불일치 (경미)

배율 x1에서 보드 캡처는 6.38%, 모델 통제실험은 0.00%다. 주행이 달라 조도
조건이 달랐을 가능성이 크나 확인되지 않았다. 같은 궤적으로 보드 재생
대조를 돌리면 바로 갈린다.

### 10. 강건성 스윕의 검출 측 절반

배율별로 고장을 주입해 검출률·검출 지연이 유지되는지 봐야 한다.
`CARLA_LIVE_VERIFY=1` 주행 후 `fault_latency_metrics.py`.
배율마다 약 10분. **오탐 곡선만으로는 강건성을 주장할 수 없다.**

### 11. MRM 절차를 RTL 에 반영 (golden 이 선행 구현됨)

골든 모델에 UNECE R157 MRM 절차 6단계가 들어갔다. **RTL 에는 아직 없다.**

```
1. Hazard ON      2. Brake level 3      3. 다운시프트
4. 완전 정차      5. 정차 유지          6. 정차 10초 뒤 사람 개입 시 Manual
```

`risk_control.sv` 646-654 행은 1/2/3 만 있고 4/5/6 이 없다. 구체적 차이:

| 항목 | RTL | 골든 |
|---|---|---|
| `mrm` | `td_remain_sec == 0` 순간 조건 | 진입 후 절차 끝까지 래치 |
| 다운시프트 | `rpm<=1 && can_downshift && gear>0` | `gear>0` (사양은 무조건 단계) |
| 정차 판정 | 없음 | `|speed_x| <= MRM_STOP_SPEED(10 raw)` 래치 |
| 인계 | manual_mode 즉시 해제 | 정차 후 10초 지나야 해제 |

그래서 `compare_golden_vs_pl` 은 **MRM 구간의 제어 명령·mrm·td 대조를
제외**하고 사유를 출력한다. 신뢰도 워드는 구간 안에서도 계속 비교한다.
**RTL 에 같은 절차를 넣으면 이 제외를 없애야 한다.**

### 12. 제어 명령 대조 잔여 불일치 (MRM 과 무관, 별건)

MRM 구간을 뺀 1,217표본에서 최종 가속 72건(94.08%), 최종 제동 71건(94.17%),
비상등 2건이 어긋난다. risk 워드(유효 tier)는 100% 일치하는데 명령만
어긋나므로 **tier 전환 지점의 1표본 정렬 문제**로 의심되나 규명되지 않았다.
예: `seq3332 최종제동 모델=3 보드=0`, `최종가속 모델=0 보드=10`.
제어 중재 대조는 다른 세션에서 추가된 기능이다.

### 13. 잡음 모델에 백색 성분 추가

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
  `CARLA_INJECT_FAULT` / `CARLA_INJECT_RISK` 환경변수나
  `live_scenario_verifier.py`(`CARLA_LIVE_VERIFY=1`)로 프로그램 구동해라.
- **불일치가 났다고 골든을 RTL 에 맞추지 마라.** 골든이 정답이고 RTL 이
  거기 맞춰야 한다(3절).  단 골든이 사양을 잘못 옮겼을 가능성은 항상 남아
  있으므로, 어느 쪽이 옳은지 물리·산술로 먼저 판정하라.
- **RTL 과 골든을 같은 가정으로 동시에 고치지 마라.** 그러면 둘의 일치가
  아무것도 증명하지 못한다. 2026-08-14 에 관계식 6/7/8 에서 이 실수를 했다.
- 승인은 채팅 본문이 아니라 `AskUserQuestion` 선택 창으로 받아라
  (`CLAUDE.md` 2절).

---

## 10. 최근 작업 기록

**2026-08-15** — 골든이 정답 코드로 방향 전환.  상태 판정식 교체
(`c_independent`, 지속 격상, stuck=DEGRADED), 제어 중재 이식,
`step()` 시간 자동 진행, 제어 대조 추가.  Python 측: distance 주입 고정,
timeout 주기적 통과, 속도 기준 다운시프트, 무인 주입 환경변수.
실측: stuck 41초 연속 DEGRADED 유지, 제어 대조 3452건 중 불일치 4(전부 지터).
**RTL 미반영 — 8절 A 가 다음 작업.**


전체 근거와 수치는
폐기된 참조 모델 기반이라 인용하지 마라.

**2026-08-14 1차** — 참조 모델 작성(폐기됨), RTL 결함 3건
수정(`pred_gyro_z_3`/`pred_accel_y_3` 곱셈 폭, 관계식 4 구심항 누락),
인터페이스 해상도 복원(속도 8→14비트, 조향 5→8비트, reg8 여유 비트 사용).
오탐률 accel_x 70.1%→9.6%, accel_y 55.3%→7.1%, gyro_z 27.7%→2.9%.
**임계값은 하나도 바꾸지 않았다.**

**2026-08-14 2차** — 상태 판정식 확정, TD/MRM + 유효 tier 참조 구현(폐기됨)
(비교 범위에 risk 워드/HUD/TD/MRM 추가), sample_seq 보류 방식 timeout 시험
확보, `standstill_slope` 시나리오로 관계식 9~16 최초 활성화,
`CARLA_SENSOR_NOISE_SCALE` 추가.

**2026-08-14 3차** — 환경 채널 바닥 진동 도입(온습도·조도 오탐 100/100/93%
→ 0%), 통제 실험 방식 확립(`noise_injection_sweep.py`), 강건성 곡선 산출
(한계 x2.0, INVALID은 x4까지 0건), 남은 오탐의 주범이
`NOISE_THRESHOLD_2`(부호 반전)임을 수치 일치로 규명.
`test_scenario_pl_alignment` 에 바닥 진동 성질 시험 신규 추가(26건).

**미커밋 상태다.** RTL 3개 + AXI IP 1개 + Python 다수 + 문서.
