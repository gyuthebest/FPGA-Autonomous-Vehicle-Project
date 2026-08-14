"""실보드 UDP 브리지 왕복 시험 (CARLA 불필요).

보드를 프로그래밍한 직후, CARLA를 켜기 전에 실행한다.
UDP 경로 / AXI 커밋 / PL 파이프라인이 실제로 동작하는지 최소한으로 확인하고,
받은 신뢰도 워드를 골든 모델과 즉시 대조한다.

  python board_smoke_test.py                     기본 192.168.1.10:5001
  python board_smoke_test.py --ip 192.168.1.10   주소 지정
  python board_smoke_test.py --frames 40         프레임 수 지정

--replay 로 실캡처의 REG 벡터를 그대로 재생할 수도 있다.

  python board_smoke_test.py --replay logs/pl_verification/pl_capture_*.csv

합성 프레임은 자극이 단조로워(distance 램프 외에는 정지값) accel/gyro
consistency 경로를 거의 건드리지 않는다.  재생 모드는 실제 주행 자극을
쓰면서도 CARLA 를 루프에서 빼기 때문에 호스트 지터가 없다.  즉
"보드가 골든 모델과 비트 단위로 같은가"를 판정하기에 가장 깨끗한 경로다.
(CARLA 를 물린 캡처는 100 ms 를 넘는 송신 간격이 생기면 PL 에 실제
 transport timeout 증거가 쌓여 디바운스 위상이 어긋난다.)
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from fpga_interface import FPGAInterface, build_input_words
from compare_golden_vs_pl import decode_input_words
from pl_model import CHANNEL_ORDER, PLModel


REL_NAME = {0: "NORMAL", 1: "DEGRADED", 2: "INVALID", 3: "?"}


def make_frame(seq: int, distance_m: float, closing: float, situation: int):
    """정상 주행에 가까운 무해한 프레임."""
    return build_input_words(
        sample_seq=seq,
        accel_xyz=(0.0, 0.0, 9.81), gyro_xyz=(0.0, 0.0, 0.0),
        incline_xyz=(0.0, 0.0, 0.0), speed_xyz=(20.0, 0.0, 0.0),
        distance_m=distance_m, approach_speed_mps=closing,
        temperature=22.0, humidity_pct=40.0, lux=30000.0,
        speed_limit_kmh=50.0, weather=0, rpm_level=2,
        accelerator=8, brake=0, steering_normalized=0.0,
        manual_mode=False, gear=2, headlight=False, hazard=False,
        situation=situation,
    )


def run_timeout_test(fpga, args) -> int:
    """transport timeout 경로를 결정론적으로 시험한다.

    핵심 착상: **프레임은 계속 보내되 sample_seq 를 올리지 않는다.**
    preprocessor 의 valid_s0 = (sample_seq_in != sample_seq_out) 이므로
    PL 은 "새 표본이 없다"고 판단하고 timeout_phase_cnt 가 100 ms 마다
    raw_timeout 을 한 번씩 낸다.  UDP 를 끊거나 지터를 기다릴 필요가 없고,
    호스트가 지속 시간을 정확히 통제한다.

    **관측 지점 주의.**  AXI 로 나오는 신뢰도 워드(read_reg10)는
    risk_control 의 rel_out 이고, 이것은 valid_in_rel 에서만 래치된다.
    따라서 보류하는 동안에는 워드가 갱신되지 않고 마지막 값이 그대로 보인다.
    확정된 timeout 은 **재개 첫 표본**에서 처음 드러난다.  실측:

        보류 5초 동안        rel = 0x150000  (갱신 없음)
        재개 +1 표본         rel = 0x2AAAAA  (11채널 전부 INVALID)
        재개 +2 표본         rel = 0x150000  (timeout_confirm_hold 해제)

    표본 단위 대조 대신 **불변조건**으로 판정한다.  경계에서 틱이 한 번 더
    들어갔는지는 복원할 수 없지만, timeout_cnt 가 TIMEOUT_N 에서 포화하므로
    충분히 긴 보류 뒤의 확정 상태는 ±1틱에 영향받지 않는다.
    """
    hold_frames = args.timeout_hold
    expected_ticks = hold_frames * 50 // 100
    print(f"\n[timeout 시험] {hold_frames} 프레임 동안 sample_seq 를 고정한다 "
          f"(약 {hold_frames * 50} ms, 예상 틱 {expected_ticks} / 확정 임계 10)")

    model = PLModel()
    seq = 0

    def send(sequence):
        words = make_frame(sequence, 200.0, 0.0, 4)
        return words, fpga.exchange(words, sequence)

    # 1) 정상 표본으로 보드와 모델을 같은 상태로 수렴시킨다
    for _ in range(args.warmup):
        seq += 1
        words, _ = send(seq)
        model.step(decode_input_words(words), sample_seq=seq, situation=4)
        time.sleep(0.05)

    seq += 1
    words, settled = send(seq)
    model.step(decode_input_words(words), sample_seq=seq, situation=4)
    if settled is None:
        print("  FAIL: 예열 중 응답 없음")
        return 1
    baseline = settled.reliability_word
    print(f"  보류 전 신뢰도 워드 = 0x{baseline:06X}")

    # 2) sample_seq 를 고정한 채 계속 보낸다 (PL 은 새 표본이 없다고 본다)
    replies = 0
    for _ in range(hold_frames):
        if send(seq)[1] is not None:
            replies += 1
        time.sleep(0.05)
    print(f"  보류 중 응답 {replies}/{hold_frames} "
          f"(워드는 valid 표본에서만 래치되므로 갱신되지 않는 것이 정상)")
    for _ in range(expected_ticks):
        model.tick_missing_sample()

    # 3) 재개 -- 첫 표본에서 확정된 timeout 이 드러난다
    seq += 1
    words, resumed = send(seq)
    expected = model.step(decode_input_words(words), sample_seq=seq, situation=4)
    if resumed is None:
        print("  FAIL: 재개 첫 표본에 응답 없음")
        return 1

    invalid_channels = sum(
        1 for position in range(len(CHANNEL_ORDER))
        if ((resumed.reliability_word >> (position * 2)) & 0x3) == 2)
    model_invalid = sum(1 for name in CHANNEL_ORDER if expected.state[name] == 2)
    print(f"  재개 +1 신뢰도 워드 = 0x{resumed.reliability_word:06X}  "
          f"INVALID {invalid_channels}/{len(CHANNEL_ORDER)}  "
          f"(골든 모델 {model_invalid}/{len(CHANNEL_ORDER)})")

    # 4) 복구까지 걸리는 표본 수 (기준선으로 돌아오면 복구)
    recovered_at = None
    for step in range(2, 42):
        time.sleep(0.05)
        seq += 1
        words, result = send(seq)
        model.step(decode_input_words(words), sample_seq=seq, situation=4)
        if result is not None and result.reliability_word == baseline:
            recovered_at = step
            break
    print(f"  재개 후 기준선 복귀까지 {recovered_at if recovered_at else '>40'} 표본")

    failures = []
    if invalid_channels != len(CHANNEL_ORDER):
        failures.append(f"재개 첫 표본의 INVALID 채널이 {invalid_channels}개다 "
                        f"(전 채널이어야 한다)")
    if model_invalid != invalid_channels:
        failures.append(f"골든 모델({model_invalid})과 보드({invalid_channels})의 "
                        f"INVALID 채널 수가 다르다")
    if recovered_at is None:
        failures.append("재개 후 40표본 안에 기준선으로 돌아오지 않았다")

    if failures:
        print("\n결과: FAIL")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("\n결과: PASS - sample_seq 보류만으로 transport timeout 확정과 복구가 "
          "재현되고, 골든 모델과 일치한다.")
    return 0


def load_replay_words(path: Path):
    """캡처의 reg0..reg9 를 그대로 꺼낸다 (sample_seq 는 재생 시 다시 매긴다)."""
    frames = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if row.get("fpga_response_valid", "").strip().lower() not in ("1", "true"):
                continue
            frames.append([int(row[f"reg{i}_hex"], 16) for i in range(10)])
    return frames


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.1.10")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--local-port", type=int, default=5002)
    parser.add_argument("--timeout-ms", type=float, default=200.0)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=20,
                        help="비교에서 제외할 예열 프레임 수")
    parser.add_argument("--replay", type=Path,
                        help="실캡처 CSV 의 REG 벡터를 그대로 재생한다")
    parser.add_argument("--timeout-test", action="store_true",
                        help="sample_seq 를 고정해 transport timeout 을 재현한다")
    parser.add_argument("--timeout-hold", type=int, default=40,
                        help="sample_seq 고정 프레임 수 (기본 40 = 약 2초)")
    args = parser.parse_args()

    replay_frames = None
    if args.replay:
        if not args.replay.exists():
            sys.exit(f"캡처 파일이 없다: {args.replay}")
        replay_frames = load_replay_words(args.replay)
        if not replay_frames:
            sys.exit(f"재생할 프레임이 없다: {args.replay}")
        args.frames = len(replay_frames)

    print("=" * 70)
    print(f"보드 UDP 왕복 시험  {args.ip}:{args.port}")
    if replay_frames:
        print(f"재생 : {args.replay.name}  ({len(replay_frames)} 프레임)")
    print("=" * 70)

    fpga = FPGAInterface(board_ip=args.ip, board_port=args.port,
                         local_port=args.local_port,
                         timeout_s=args.timeout_ms / 1000.0, enabled=True)

    if args.timeout_test:
        try:
            return run_timeout_test(fpga, args)
        finally:
            fpga.close()

    print(f"예열 {args.warmup} 프레임은 비교에서 제외한다.")
    print("  보드는 시험 시작 전까지 표본을 받지 못한 상태라 transport timeout이")
    print("  확정되어 있고(전 채널 INVALID), 복구 첫 표본은 timeout_mask_1s로")
    print("  noise 이력에서 제외된다. 골든 모델에는 그 사전 상태가 없으므로")
    print("  양쪽이 수렴할 때까지 기다린다. (timeout 치유 5표본 + noise 창 10표본)")

    model = PLModel()
    responses = 0
    timeouts = 0
    seq_errors = 0
    mismatches = []
    roundtrips = []

    total_frames = args.warmup + args.frames
    try:
        for index in range(total_frames):
            seq = index + 1
            warming = index < args.warmup
            # 첫 10프레임은 무표적, 이후 접근 -> situation 001 한 번 발생
            step = index - args.warmup
            if replay_frames:
                # 예열 구간은 첫 프레임을 반복해 보드/모델을 같은 상태로 맞춘다.
                source = replay_frames[max(0, step)]
                words = list(source)
                words[9] = seq                      # sample_seq 만 다시 매긴다
            else:
                if warming or step < 10:
                    distance, closing, situation = 200.0, 0.0, 4
                elif step == 10:
                    distance, closing, situation = 60.0, 20.0, 1
                else:
                    distance, closing, situation = max(8.0, 60.0 - (step - 10)), 20.0, 4
                words = make_frame(seq, distance, closing, situation)

            sample = decode_input_words(words)
            situation = sample.get("situation", 0)
            expected = model.step(sample, sample_seq=seq, situation=situation)

            start = time.perf_counter()
            result = fpga.exchange(words, seq)
            elapsed_ms = (time.perf_counter() - start) * 1000.0

            if result is None:
                if not warming:
                    timeouts += 1
                continue
            if warming:
                time.sleep(0.05)
                continue
            responses += 1
            roundtrips.append(elapsed_ms)

            if result.sample_seq != seq:
                seq_errors += 1
                continue

            for position, name in enumerate(CHANNEL_ORDER):
                board_state = (result.reliability_word >> (position * 2)) & 0x3
                if board_state != expected.state[name]:
                    if len(mismatches) < 15:
                        mismatches.append(
                            f"seq{seq} {name}: 모델={REL_NAME[expected.state[name]]} "
                            f"보드={REL_NAME.get(board_state, '?')}"
                        )
            time.sleep(0.05)          # 20 Hz
    finally:
        fpga.close()

    print(f"\n전송 {args.frames}  응답 {responses}  timeout {timeouts}  "
          f"sequence 불일치 {seq_errors}")
    if roundtrips:
        roundtrips.sort()
        print(f"왕복 지연  평균 {sum(roundtrips)/len(roundtrips):.2f} ms  "
              f"최소 {roundtrips[0]:.2f}  최대 {roundtrips[-1]:.2f}  "
              f"p95 {roundtrips[int(len(roundtrips)*0.95) - 1]:.2f}")

    if responses == 0:
        print("\n결과: FAIL - 보드가 한 프레임도 응답하지 않았다.")
        print(f"  마지막 오류: {fpga.last_error}")
        print("  확인할 것:")
        print("   1) 보드 IP/서브넷  (호스트가 같은 대역에 있어야 한다)")
        print("   2) A53 브리지가 실행 중인지 (program_and_bringup 재실행)")
        print("   3) 방화벽이 UDP 5002 수신을 막고 있는지")
        return 1

    if timeouts or seq_errors or mismatches:
        print(f"\n결과: 문제 있음")
        if timeouts:
            print(f"  - timeout {timeouts}프레임")
        if seq_errors:
            print(f"  - sequence 불일치 {seq_errors}프레임")
        for item in mismatches:
            print(f"  - {item}")
        return 1

    print("\n결과: PASS - 전 프레임 응답, sequence 일치, "
          "신뢰도 판정이 골든 모델과 완전히 일치한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
