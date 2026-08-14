# situation 신호 정정, 타이밍 클로저, 보드 준비 완료 (2026-08-14)

## 0. 결론

| 항목 | 결과 |
|---|---|
| **구현 setup WNS** | **+0.118 ns** (TNS 0, failing endpoint 0) |
| **구현 hold WHS** | **+0.016 ns** (THS 0, failing endpoint 0) |
| DRC error / critical warning | **0 / 0** |
| bitstream / XSA | 생성됨 (5.57 MB / 1.06 MB) |
| LUT / FF / DSP / BRAM | 18.07% / 6.60% / 10.00% / 0% |
| 오프라인 검증 전체 | **OVERALL: PASS** |

타이밍은 **통과**했다. 다만 여유가 이전(+0.280 ns)보다 줄어 **+0.118 ns**다.
자세한 내용은 3절.

---

## 1. 지난 보고 정정 — 관계식 14 마스크는 옳았다

**이전 보고에서 다음과 같이 적었고, 이는 틀렸다.**

> `consistency_mask_4 = (situation != 0)`은 "정지 상태"가 아니라 situation 값
> 기반이라 정상 선회 중에도 활성화된다. 설계 의도 문제다.

`situation` 인코딩이 `000 = 정지`이므로 `situation != 000`은 정확히
**"정지가 아니면 마스크"**를 뜻한다. 관계식 14("정지 중이면 요각속도는 0")는
정지 상태에서만 활성화된다. **RTL 설계는 처음부터 옳았다.**

진짜 원인은 내가 만든 시험 벡터였다. `make_trace_vectors.py`가
`state.situation = 0`을 하드코딩한 채 0.5 rad/s 선회를 생성해서,
PL에 **"정지 중"이라고 알리면서 회전하는 모순된 입력**을 넣고 있었다.
PL은 그 모순을 정확히 잡아낸 것이다.

`main.py`의 situation 인코딩은 이미 사양대로 구현되어 있었다.

### 수정

- `make_trace_vectors.py` — situation을 하드코딩하지 않고 상태에서 유도한다.
  수정 후 벡터의 situation 분포:

  | 시나리오 | situation |
  |---|---|
  | turn | 010 (자세변화, gyro_z 0.5 > 0.340) |
  | straight | 100 (정상) |
  | brake_ice | 001 (장애물 등장) → 100 (정상) |

---

## 2. situation 인코딩 — 사양 반영 및 시험 추가

```
000 정지        (<= 1 km/h)      -> consistency_mask_4/5/6 열림
001 장애물 등장  (200 m sentinel 통과) -> distance/approach_speed jump 마스크
010 자세변화    (각속도 x/y/z)   -> distance/approach_speed jump 마스크
011 날씨 변화                    -> 온도/습도/조도 jump 마스크
100 정상 주행                    -> 마스크 없음
```

### 2.1 발견한 사양 미준수 — gyro_x 누락

자세변화 판정이 `gyro_y`, `gyro_z`만 보고 있었다. 사양은 **x/y/z 세 축**이다.
급격한 롤(roll) 변화가 자세변화로 보고되지 않았다. `gyro_x`를 추가했다.

### 2.2 테스트 가능하도록 분리

situation 판정이 main 루프 안에 인라인으로 있어 검증이 불가능했다.
`classify_situation()` 함수로 분리하고 상수를 이름 붙여 노출했다
(`SITUATION_STOPPED` … `SITUATION_NORMAL`).

**신규 단위시험 8건** (전부 PASS):

- 정지 / 정상 주행 기본 분류
- 장애물 등장은 **sentinel 통과 순간에만** (이미 추적 중이면 사건 아님)
- 자세변화가 **x/y/z 세 축 모두**에 반응
- 임계 경계 (0.340 통과 / 0.341 자세변화)
- 날씨 변화가 최우선
- 사건(010)이 상태(000)보다 우선
- 모든 코드가 3비트 범위 내

---

## 3. 타이밍 클로저

### 3.1 결과

```
setup  WNS +0.118 ns   TNS 0.000   failing endpoints 0
hold   WHS +0.016 ns   THS 0.000   failing endpoints 0
DRC    error 0, critical warning 0
```

### 3.2 임계 경로는 gyro 확장이 아니다

```
Source:      u_axi_slave/slv_reg3_reg[0]
Destination: u_preprocessor/pred_data_out_reg[pred_accel_y_1][11]
Logic Levels: 34 (DSP48 3단 연쇄 + CARRY8 3 + LUT 13)
```

**`pred_accel_y_1`** 경로다. `pred_accel_y_*`는 tan LUT와 곱셈이 3단으로
연쇄되며, 이는 내가 건드리지 않은 **기존 구조**다. `pred_gyro_*_1`을
16 → 28비트로 넓힌 변경은 임계 경로에 나타나지 않는다.

### 3.3 주의 — 여유가 얇다

이전 빌드 +0.280 ns → 이번 **+0.118 ns**. 11.25 ns 주기 대비 약 1%다.
통과이긴 하나 다음 조건에서 쉽게 음수가 될 수 있다.

- 로직을 조금이라도 더 추가할 때
- 다른 speed grade / 온도 등급 부품
- Vivado 버전 변경

**권장 후속 조치**: codex가 보고한 DRC 경고 67건(DPIP-2 26, DPOP-3 17,
DPOP-4 24)이 정확히 이 경로를 가리킨다. DSP48 입출력 파이프라인 레지스터를
켜면 여유가 크게 늘어난다. 다만 **latency가 바뀌므로 valid 정렬을 함께
수정**해야 하고, 회귀 전체를 다시 돌려야 한다. 지금 당장은 통과하므로
보드 검증을 먼저 진행하고, 여유 확보는 별도 작업으로 다루기를 권한다.

### 3.4 빌드에 하드 게이트 추가

`build_full_project_88888mhz.tcl`이 타이밍 리포트를 파일로만 남기고 통과 여부를
판정하지 않았다. 이제 다음을 출력하고 **음수 슬랙이면 빌드를 실패시킨다.**

```
TIMING_SETUP_WNS / TIMING_SETUP_TNS / TIMING_HOLD_WHS / TIMING_HOLD_THS
TIMING_CLOSURE=PASS|FAIL
DRC_ERRORS / DRC_CRITICAL_WARNINGS
```

타이밍이 깨진 bitstream이 보드에 올라가는 일은 이제 구조적으로 막힌다.

---

## 4. 골든 모델 최종 상태

situation 수정 후 3 시나리오 전부 재대조:

| 시나리오 | 결과 |
|---|---|
| turn | PASS |
| straight | PASS |
| brake_ice | PASS |

gyro consistency도 전 표본 OK가 되었다.

```
부호 반전(wrap) 징후 없음. -> 비트폭은 충분하다.
잔차 최대 6752 / 평균 2829
TH_GYR=7300 은 양자화 바닥(1787)과 실측 최대(6752)를 모두 넘는다. 적절하다.
```

`analyze_pl_trace.py`가 TH_GYR을 하드코딩하지 않고 **RTL에서 직접 읽도록**
바꿨다. 상수를 바꿔도 분석 도구가 낡지 않는다.

### 모델 커버리지 (정직한 한계)

| 항목 | 상태 |
|---|---|
| preprocessor delta / pred(gyro, distance) | 구현 |
| range / jump / stuck / noise / timeout | 구현 |
| consistency 관계식 6/7/8, 14 | 구현 |
| consistency 관계식 17(steer), accel/distance | **미구현** |
| reliability 상태 | gyro_z 제외 구현 |
| risk_types 분류 | 구현 |
| risk_control 제동 블렌딩 | 구현 |
| risk_control TD/MRM 타이머 | **미구현** |

`rel_gyro_z`는 관계식 17(조향 기반 예상 요각속도)에 의존해 비교에서 제외했다.
나머지 21개 비교량으로 판정한다.

---

## 5. 보드 반입 시 실행

```bash
sources_1\verification\board_arrival.bat
```

1. 오프라인 회귀 게이트 (실패 시 하드웨어를 건드리지 않고 중단)
2. packaged IP 갱신 + 재빌드 + **타이밍 게이트**
3. 보드 프로그래밍 + A53 브리지 bring-up
4. 브리지 수신 상태 확인

이미 빌드가 끝나 있으므로 `board_arrival.bat skipbuild`로 2단계를 건너뛸 수
있다.

이후 CARLA 실행 및 라이브 검증은
[BOARD_BRINGUP_RUNBOOK.md](BOARD_BRINGUP_RUNBOOK.md) 3~8단계를 따른다.

---

## 6. 라이브 검증 수행 방식에 대한 한계 명시

요청하신 "직접 인터페이스를 조작하며 모든 경우 재현"은 **GUI를 손으로
조작하는 방식으로는 수행할 수 없다.** pygame 창을 직접 클릭/키입력할 수 있는
도구가 없기 때문이다.

대신 `live_scenario_verifier.py`(`CARLA_LIVE_VERIFY=1`)를 사용한다. 이쪽이
검증으로서는 더 낫다.

- 제어판 상태를 **프로그램으로** 순차 주입하므로 재현 가능하다
- 시나리오마다 화면을 자동 캡처한다
- 최종 5연속 표본을 자동 판정한다
- 실패한 케이스만 골라 재실행할 수 있다

내가 할 수 있는 것: 스크립트 실행, 로그/캡처 수집, 골든 모델 대조, 실패
원인 분석, 보고서 작성. 할 수 없는 것: 마우스로 슬라이더를 끄는 조작.
