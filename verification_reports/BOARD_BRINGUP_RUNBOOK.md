# 보드 반입 후 실행 순서 (Runbook)

보드를 연결하기만 하면 아래 순서대로 검증 결과까지 나오도록 준비해 두었다.
각 단계는 실패 시 다음으로 넘어가지 말고 원인을 먼저 규명한다.

---

## 사전 확인 (보드 없이 지금 가능)

```bash
sources_1\verification\run_offline_verification.bat
```

`OVERALL: PASS` 여야 한다. 현재 상태:

| 항목 | 결과 |
|---|---|
| tb_pl_full_verification | 88 PASS / 0 FAIL |
| tb_carla_axi_replay | 5 samples / 0 FAIL |
| tb_risk_reliability_matrix | 121 PASS / 0 FAIL |
| 골든 모델 vs RTL (turn/straight/brake_ice) | 3960 비교 항목 / 0 불일치 |
| verify_sensor_noise | 10개 항목 PASS |
| test_scenario_pl_alignment | 17 PASS |

---

## 1단계 — 재합성 및 bitstream 생성

RTL이 세 군데 바뀌었으므로 반드시 다시 빌드해야 한다
(`pred_gyro_*_1` 28비트, `TH_GYR` 7300, 제동 블렌딩).

```bash
sources_1\verification\build_full_project_88888mhz.bat
```

**합격 기준**

- 합성 / 구현 / bitstream / XSA 생성 PASS
- setup WNS > 0, hold WHS > 0, failing endpoint 0
- DRC error 0, critical warning 0

> 주의: `pred_gyro_*_1`이 16 -> 28비트로 넓어졌다. consistency 비교 경로가
> 길어져 **timing이 이전(WNS +0.280 ns)보다 나빠질 수 있다.** WNS가 음수면
> 그 경로에 파이프라인 단을 넣기 전에 먼저 보고할 것.

산출물 SHA-256을 기록해 두면 이후 단계에서 어떤 bitstream을 올렸는지 추적된다.

## 2단계 — 보드 프로그래밍 및 bring-up

```bash
sources_1\verification\program_and_bringup.tcl
```

**합격 기준**: PS가 A53에서 `ps_carla_bridge`를 실행하고 UDP 5001 포트에서
수신 대기.

```bash
sources_1\verification\probe_a53_state.tcl
```

## 3단계 — 개루프 캡처 (FPGA on, 차량 제어는 Python)

가장 먼저 할 검증이다. PL 출력이 차량에 적용되지 않으므로 인과가 깨끗하다.

```bash
set FPGA_ENABLED=1
set CARLA_MAP=Town04
set PL_VERIFY_LOG=1
python CARLA_FPGA_PROJECT\main.py
```

제어판에서 **Apply FPGA Output을 끈 상태**로 3~5분 정상 주행한다.
정지 / 직선 가감속 / 좌우 회전 / 장애물 접근·이탈을 각각 포함시킨다.

산출물: `CARLA_FPGA_PROJECT/logs/pl_verification/pl_capture_<timestamp>.csv`

## 4단계 — 골든 모델 대조 (핵심 판정)

```bash
python CARLA_FPGA_PROJECT\compare_golden_vs_pl.py --board CARLA_FPGA_PROJECT\logs\pl_verification\pl_capture_<timestamp>.csv
```

**이 단계가 "FPGA가 우리 로직을 그대로 반영했는가"의 판정이다.**

보드가 AXI로 내보내는 신뢰도 워드(read_reg10, 채널 11개 x 2비트)를 골든
모델의 판정과 표본마다 대조한다. `PASS`면 실보드 신뢰도 판정이 소프트웨어
기준 구현과 완전히 일치한다는 뜻이다.

**불일치가 나오면** 원인은 셋 중 하나이며 반드시 규명한다.

1. RTL이 의도한 알고리즘과 다르게 구현됨
2. 합성/타이밍/AXI 전송 문제
3. 골든 모델이 사양을 잘못 옮김

구분 방법: 같은 캡처의 REG 벡터를 xsim에 재생해 본다.

```bash
sources_1\verification\run_pl_trace.bat <캡처에서_뽑은_벡터.csv>
python CARLA_FPGA_PROJECT\compare_golden_vs_pl.py --vectors <벡터.csv>
```

- 시뮬레이션은 **일치**하는데 보드만 불일치 -> **원인 2** (하드웨어/타이밍)
- 시뮬레이션도 **불일치** -> **원인 1 또는 3** (로직/모델)

## 5단계 — 클럭 단위 원인 분석

4단계에서 불일치가 나온 표본을 지목해 판단 과정을 펼친다.

```bash
sources_1\verification\run_pl_trace.bat <벡터.csv> vcd
python CARLA_FPGA_PROJECT\analyze_pl_trace.py --changes
python CARLA_FPGA_PROJECT\analyze_pl_trace.py --clocks <시작> <끝>
python CARLA_FPGA_PROJECT\analyze_pl_trace.py --gyro
python CARLA_FPGA_PROJECT\analyze_pl_trace.py --brake
```

파형이 필요하면 `verification_reports/pl_trace.vcd`를 Vivado로 연다.

## 6단계 — 폐루프 실주행

Apply FPGA Output을 켜고 같은 시나리오를 반복한다.
검증 질문은 *"3단계 개루프 판단과 같은가"* 이지 *"Python 컨트롤러와 같은가"*
가 아니다.

## 7단계 — 라이브 시나리오 회귀 (74건)

```bash
set CARLA_LIVE_VERIFY=1
python CARLA_FPGA_PROJECT\main.py
```

codex가 통과시킨 74개 시나리오를 재확인한다. 이번 변경으로 **결과가 달라질
것으로 예상되는 항목**을 미리 적어 둔다.

| 시나리오 | 이전 | 예상 |
|---|---|---|
| BLACK ICE 노면 | 온도 -60 degC 주입 | **-8.0 degC 주입** (스케일 수정) |
| ICE/BLACK ICE + 충돌 | brake 0 | **brake 5 / 3** (블렌딩) |
| 횡방향 DANGER + 충돌 | brake 0 | **brake 5** |
| 정상 주행 신뢰도 | temp/hum/lux DEGRADED | **NORMAL** (잡음 추가) |
| 선회 중 gyro_z consistency | 항상 오류 | 개선 (TH_GYR 7300) |
| distance range 버튼 | 없음 | **신규 동작** (250 m 주입) |

## 8단계 — 정량 지표 산출

프로젝트 주장("신뢰도 및 위험도 기반")을 뒷받침하는 수치다.
현재는 PASS/FAIL만 있고 지연·오탐률이 없다.

- 채널별 **검출 지연**(고장 주입 -> 확정까지 표본 수)
- **오탐률**(정상 주행 중 확정 고장 / 전체 표본)
- **복구 지연**(고장 해제 -> NORMAL 복귀까지 표본 수)

`analyze_pl_capture.py`와 `compare_golden_vs_pl.py` 출력으로 산출한다.

---

## 아직 열려 있는 항목

| 항목 | 상태 |
|---|---|
| 관계식 14 (gyro_z 정지 기준)의 마스크 | `consistency_mask_4 = (situation != 0)` 이라 정상 선회(situation=0) 중에도 활성화된다. 골든 모델도 같은 동작을 재현하므로 RTL 구현은 일관되나, "정지 상태"를 뜻하려면 마스크가 속도 기반이어야 한다. **설계 결정 필요** |
| 관계식 17 (steer) | 골든 모델 미구현 |
| accel/distance consistency 관계식 | 골든 모델 미구현 |
| risk_control TD/MRM 타이머 | 골든 모델 미구현 |
| Jump/Noise 임계값 | 실캡처 기반 산정 미완 |
