"""UDP bridge between the CARLA laptop and the Zynq PS/PL design.

Python sends ten AXI write-register images to the PS. The PS writes REG0..REG8
first and REG9 (sample_seq) last, committing one coherent sample to the PL.
All UDP words use network byte order. The returned REG11 and REG12 sequence
numbers must both match the request before CARLA accepts a result.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import socket
import struct
import time
from typing import Iterable, Optional, Sequence, Tuple


INPUT_WORD_COUNT = 10
OUTPUT_WORD_COUNT = 15
INPUT_PACKET = struct.Struct("!10I")
OUTPUT_PACKET = struct.Struct("!15I")


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, int(value)))


def _quantize_signed(value: float, scale: float, bits: int) -> int:
    raw = int(round(float(value) * scale))
    return _clamp(raw, -(1 << (bits - 1)), (1 << (bits - 1)) - 1)


def _quantize_unsigned(value: float, scale: float, bits: int) -> int:
    raw = int(round(float(value) * scale))
    return _clamp(raw, 0, (1 << bits) - 1)


def _twos(value: int, bits: int) -> int:
    return int(value) & ((1 << bits) - 1)


def _signed(value: int, bits: int) -> int:
    value &= (1 << bits) - 1
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def _compressed_signed(value: int, shift: int, bits: int) -> int:
    compressed = int(value) >> shift
    compressed = _clamp(compressed, -(1 << (bits - 1)), (1 << (bits - 1)) - 1)
    return _twos(compressed, bits)


def _compressed_unsigned(value: int, shift: int, bits: int) -> int:
    return _clamp(int(value) >> shift, 0, (1 << bits) - 1)


@dataclass(frozen=True)
class FPGAResult:
    sample_seq: int
    accelerator: int
    brake: int
    steering_raw: int
    gear: int
    headlight: bool
    hazard: bool
    manual_mode: bool
    speed_limit_kmh: float
    transition_demand: bool
    hud_warning: bool
    mrm: bool
    td_remain_sec: int
    risk_word: int
    reliability_word: int

    @property
    def steering_normalized(self) -> float:
        return _clamp(self.steering_raw, -100, 100) / 100.0


def build_input_words(
    *, sample_seq: int, accel_xyz: Sequence[float], gyro_xyz: Sequence[float],
    incline_xyz: Sequence[float], speed_xyz: Sequence[float], distance_m: float,
    approach_speed_mps: float, temperature: float, humidity_pct: float,
    lux: float, speed_limit_kmh: float, weather: int, rpm_level: int,
    accelerator: int, brake: int, steering_normalized: float,
    manual_mode: bool, gear: int, headlight: bool, hazard: bool, situation: int,
) -> Tuple[int, ...]:
    if not all(len(v) == 3 for v in (accel_xyz, gyro_xyz, incline_xyz, speed_xyz)):
        raise ValueError("xyz inputs must each contain exactly three values")

    ax, ay, az = (_quantize_signed(v, 100.0, 12) for v in accel_xyz)
    gx, gy, gz = (_quantize_signed(v, 1000.0, 16) for v in gyro_xyz)
    ix, iy, iz = (_quantize_signed(v, 100.0, 16) for v in incline_xyz)
    sx, sy, sz = (_quantize_signed(v, 100.0, 14) for v in speed_xyz)
    # distance 필드는 15비트 부호 없음(0..32767 = 0..327.67 m)이고 PL의
    # RANGE_THRESHOLD_MAX는 20000(200.00 m)이다.  즉 20001..32767은 표현
    # 가능하면서 PL이 range fault로 판정하는 구간이다.  기존의 200.0 m
    # 포화는 이 구간을 Python 쪽에서 막아버려서 distance range 고장 주입을
    # 불가능하게 만들었다.  포화 지점을 필드 상한으로 올린다.  레이더 실측은
    # main.py에서 이미 200 m로 제한되므로 정상 주행 동작은 변하지 않고,
    # sentinel(정확히 20000)도 그대로 유지된다.
    distance_q = _quantize_unsigned(float(distance_m), 100.0, 15)
    approach_q = _quantize_signed(approach_speed_mps, 100.0, 13)
    # 온도 LSB는 0.1 degC 다.  risk_types.sv의 노면 분류가
    #     temperature <= -50  -> Black Ice
    #     temperature <=   0  -> Ice
    # 이고 sensor_reliability.sv의 range가 -500..600 이다.  즉 raw -500..600은
    # -50.0..60.0 degC, Black Ice 임계는 -5.0 degC 를 뜻한다.  scale=1.0 이면
    # Black Ice 임계가 -50 degC가 되어 물리적으로 도달 불가능해진다.
    temperature_q = _quantize_signed(temperature, 10.0, 11)
    humidity_q = _quantize_unsigned(humidity_pct, 1.0, 7)
    lux_q = _quantize_unsigned(lux, 1.0, 18)
    speed_limit_q = _quantize_unsigned(speed_limit_kmh, 100.0, 13)
    steering_q = _quantize_signed(steering_normalized, 100.0, 8)

    reg0 = (_twos(ay, 16) << 16) | _twos(ax, 16)
    reg1 = (_twos(gx, 16) << 16) | _twos(az, 16)
    reg2 = (_twos(gz, 16) << 16) | _twos(gy, 16)
    reg3 = (_twos(iy, 16) << 16) | _twos(ix, 16)
    reg4 = (_compressed_signed(sy, 6, 8) << 24) | (_compressed_signed(sx, 6, 8) << 16) | _twos(iz, 16)
    reg5 = (_clamp(humidity_q, 0, 0x7F) << 25) | (_compressed_signed(approach_q, 3, 10) << 15) | distance_q
    reg6 = (_clamp(int(weather), 0, 3) << 30) | (_clamp(int(accelerator), 0, 15) << 26) | (_compressed_signed(sz, 6, 8) << 18) | lux_q
    reg7 = (_clamp(int(gear), 0, 3) << 30) | (_clamp(int(rpm_level), 0, 3) << 28) | (_clamp(int(brake), 0, 15) << 24) | (_compressed_signed(steering_q, 3, 5) << 19) | (_compressed_unsigned(speed_limit_q, 5, 8) << 11) | _twos(temperature_q, 11)
    # reg8 상위는 비어 있었다.  속도의 하위 6비트를 여기로 보내 PL이 보는
    # 해상도를 0.64 m/s -> 0.01 m/s 로 되돌린다.  PL 은
    #   speed_x = $signed({slv_reg4[23:16], slv_reg8[11:6]})
    # 로 되조립하므로 상위 8비트(_compressed_signed)와 합치면 sx 원값이 된다.
    # 이 6비트가 없으면 pred_accel_*_1 의 기준값이 640(6.4 m/s^2) 계단으로만
    # 움직여 관계식 3/4/5(임계 76)가 정상 주행에서 통과할 수 없다.
    # 조향도 같은 이유로 5비트(LSB = full lock 의 8%)만 실려 있었다.
    #   steering = $signed({slv_reg7[23:19], slv_reg8[26:24]})
    reg8 = ((steering_q & 0x7) << 24) \
        | ((sz & 0x3F) << 18) | ((sy & 0x3F) << 12) | ((sx & 0x3F) << 6) \
        | (_clamp(int(situation), 0, 7) << 3) | (int(bool(hazard)) << 2) \
        | (int(bool(headlight)) << 1) | int(bool(manual_mode))
    reg9 = int(sample_seq) & 0xFFFFFFFF
    return tuple(word & 0xFFFFFFFF for word in (reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9))


def decode_output_words(words: Sequence[int], expected_seq: Optional[int] = None) -> FPGAResult:
    if len(words) != OUTPUT_WORD_COUNT:
        raise ValueError(f"expected {OUTPUT_WORD_COUNT} output words, got {len(words)}")
    risk_seq = int(words[11]) & 0xFFFFFFFF
    reliability_seq = int(words[12]) & 0xFFFFFFFF
    if risk_seq != reliability_seq:
        raise ValueError(f"pipeline sequence mismatch: risk={risk_seq}, reliability={reliability_seq}")
    if expected_seq is not None and risk_seq != (int(expected_seq) & 0xFFFFFFFF):
        raise ValueError(f"stale FPGA response: expected={expected_seq}, received={risk_seq}")
    command = int(words[13])
    return FPGAResult(
        sample_seq=risk_seq, accelerator=(command >> 9) & 0xF,
        brake=(command >> 13) & 0xF, steering_raw=_signed((command >> 17) & 0xFF, 8),
        gear=(command >> 25) & 0x3, headlight=bool((command >> 7) & 1),
        hazard=bool((command >> 8) & 1), manual_mode=bool((command >> 27) & 1),
        speed_limit_kmh=(int(words[14]) & 0x1FFF) / 100.0,
        transition_demand=bool(command & 1), hud_warning=bool((command >> 1) & 1),
        mrm=bool((command >> 2) & 1), td_remain_sec=(command >> 3) & 0xF,
        risk_word=int(words[9]) & 0xFFFF, reliability_word=int(words[10]) & 0x003FFFFF,
    )


class FPGAInterface:
    def __init__(self, board_ip: str = "192.168.1.10", board_port: int = 5001,
                 local_port: int = 5002, timeout_s: float = 0.020,
                 enabled: bool = True) -> None:
        self.board_address = (board_ip, int(board_port))
        self.timeout_s = max(0.001, float(timeout_s))
        self.enabled = bool(enabled)
        self.connected = False
        self.timeout_count = 0
        self.last_error = "disabled"
        self._socket: Optional[socket.socket] = None
        if self.enabled:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.bind(("0.0.0.0", int(local_port)))
            self._socket.settimeout(self.timeout_s)
            self.last_error = "waiting for first FPGA response"

    @classmethod
    def from_environment(cls) -> "FPGAInterface":
        enabled = os.getenv("FPGA_ENABLED", "1").strip().lower() not in {"0", "false", "no", "off"}
        return cls(os.getenv("FPGA_BOARD_IP", "192.168.1.10"),
                   int(os.getenv("FPGA_BOARD_PORT", "5001")),
                   int(os.getenv("FPGA_LOCAL_PORT", "5002")),
                   float(os.getenv("FPGA_TIMEOUT_MS", "20")) / 1000.0, enabled)

    def exchange(self, words: Iterable[int], sample_seq: int) -> Optional[FPGAResult]:
        if not self.enabled or self._socket is None:
            return None
        values = tuple(int(word) & 0xFFFFFFFF for word in words)
        if len(values) != INPUT_WORD_COUNT:
            raise ValueError(f"expected {INPUT_WORD_COUNT} input words, got {len(values)}")
        try:
            self._socket.sendto(INPUT_PACKET.pack(*values), self.board_address)
            deadline = time.monotonic() + self.timeout_s
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout
                self._socket.settimeout(remaining)
                packet, sender = self._socket.recvfrom(2048)
                if sender[0] != self.board_address[0] or len(packet) != OUTPUT_PACKET.size:
                    continue
                try:
                    result = decode_output_words(OUTPUT_PACKET.unpack(packet), sample_seq)
                except ValueError:
                    continue
                self.connected = True
                self.last_error = ""
                return result
        except (OSError, socket.timeout) as exc:
            self.connected = False
            self.timeout_count += 1
            self.last_error = str(exc) or "FPGA response timeout"
            return None

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
