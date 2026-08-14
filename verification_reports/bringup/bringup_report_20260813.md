# Zynq PS/PL Bring-up 보고서 (2026-08-13)

## 결과

- JTAG cable: PASS
- Hardware target: `xczu2_0`, `arm_dap_1`
- PS initialization: PASS
- PL bitstream programming: PASS
- PL isolation removal: PASS
- PL fabric reset release: PASS
- AXI register access: PASS
- Risk pipeline sequence: `0xA5A50001`
- Reliability pipeline sequence: `0xA5A50001`
- PS ELF download: PASS
- Cortex-A53 execution: PASS (`Running`)
- UDP network path: 대기 중

## 배포 생성물

- Bitstream: `FPGA_project/FPGA_project.runs/impl_1/design_1_wrapper.bit`
- Bitstream SHA-256: `DA2808268B92FE22EFDE76EE2A97D22F6101FC6409AE6E055375B81A2152E9AA`
- ELF: `vitis_workspace/carla_fpga_bridge/Debug/carla_fpga_bridge.elf`
- ELF SHA-256: `180BB59CAE1910598591AFBB595C10296EAA0CB1119435FF4844D459E2CFABE0`
- AXI base: `0x80000000`
- Board IPv4: `192.168.1.10/24`
- UDP input port: `5001`
- PL clock request: 90 MHz
- PL actual clock: 88.888 MHz

## PS 앱

- Processor: `psu_cortexa53_0`
- OS: standalone
- Network stack: lwIP 2.1.1
- Ethernet: GEM3
- UART: UART1
- Input: CARLA UDP packet `REG0..REG9`
- Commit: `REG9`를 마지막에 write
- Output: AXI `REG0..REG14`를 UDP response로 반환

## 해결한 bring-up 문제

1. Reset 상태의 A53에 `stop`을 먼저 실행하면 `APU L2 cache is held in reset` 오류가 발생했다. AXI 검사는 PSU target에서 수행하고 A53에는 `rst -processor`를 먼저 적용하도록 순서를 변경했다.
2. PSU target에서 PL 주소는 기본 memory map 보호에 걸리므로 AXI smoke test에 `mwr/mrd -force`를 사용했다.
3. 생성된 `psu_post_config`가 비어 있어 PL AXI가 reset/격리 상태로 남았다. Bitstream 직후 `psu_ps_pl_isolation_removal`과 `psu_ps_pl_reset_config`를 명시적으로 호출해 해결했다.

## Ethernet 및 CARLA 실시간 검증 결과

- 노트북 USB Ethernet: `이더넷 2`, 100 Mbps 링크 정상
- 노트북 IPv4: `192.168.1.20/24`
- 보드 IPv4: `192.168.1.10/24`
- 보드 ping: 3/3 응답, 손실 0%, RTT 1 ms 미만
- UART: `COM6`, 115200 baud
- PHY: Microchip/Micrel `KSZ9031RNX`, MDIO address 1
- PHY 협상 결과: 100 Mbps
- CARLA Python UDP 단독 시험: 3/3 응답, sample sequence 일치
- CARLA Town04 20 Hz 연속 시험: 959/959 응답, sequence mismatch 0
- UDP/PL 왕복 지연: 평균 0.386 ms, P95 0.550 ms, 최대 1.338 ms
- MRM 수동 해제 후 자동모드 연속 160프레임: 응답 160/160, MRM=0, TD=0, HUD warning=0, brake=0

### PS BSP 수정

Vitis 2022.2 기본 lwIP PHY 속도 판별 코드는 KSZ9031을 Marvell PHY 처리로 잘못
보내 `link speed=0`을 반환했다. 다음 BSP 소스에 KSZ9031 PHY ID `0x0022`와
vendor control register `0x1F`의 속도 상태 비트 판독을 추가했다.

`vitis_workspace/carla_fpga_platform/psu_cortexa53_0/standalone_domain/bsp/psu_cortexa53_0/libsrc/lwip211_v1_8/src/contrib/ports/xilinx/netif/xemacpsif_physpeed.c`

이 수정은 PS/lwIP BSP 수정이며 PL SystemVerilog와 비트스트림은 변경하지 않았다.
Vitis platform/BSP를 완전히 재생성하면 vendor BSP 소스가 덮어써질 수 있으므로
재생성 후 KSZ9031 수정 유지 여부를 확인해야 한다.
