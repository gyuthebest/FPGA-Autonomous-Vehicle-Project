#==============================================================================
# preprocessor.sv  타이밍 분석용 제약 파일
#
#   report_timing_summary 결과가 inf / 0ns / NA 로 나오는 것은
#   클럭이 정의되지 않아 Vivado 가 슬랙을 계산할 기준이 없기 때문이다.
#   이 파일은 클럭과 I/O 예산만 정의하며, 소스 파일은 건드리지 않는다.
#
#   사용법
#     1) Sources 창에서 preprocessor 를 Set as Top 으로 지정
#     2) 이 파일을 Constraints 에 추가하고 Set as Target Constraint File
#     3) Run Synthesis  ->  Open Synthesized Design  ->  Report Timing Summary
#==============================================================================


#------------------------------------------------------------------------------
# 1. 클럭 정의   <<< 실제 PL 클럭에 맞춰 CLK_PERIOD 만 수정하면 된다 >>>
#------------------------------------------------------------------------------
#   100 MHz -> 10.000        200 MHz -> 5.000
#   150 MHz ->  6.667        250 MHz -> 4.000
#------------------------------------------------------------------------------

set CLK_PERIOD 10.000
set CLK_PORT   clk

create_clock -name clk \
             -period $CLK_PERIOD \
             -waveform [list 0.000 [expr {$CLK_PERIOD / 2}]] \
             [get_ports $CLK_PORT]


#------------------------------------------------------------------------------
# 2. 포트 집합
#------------------------------------------------------------------------------

set all_in  [get_ports -filter {DIRECTION == IN}]
set data_in [remove_from_collection $all_in [get_ports $CLK_PORT]]
set all_out [get_ports -filter {DIRECTION == OUT}]


#------------------------------------------------------------------------------
# 3. I/O 지연 예산
#
#   preprocessor 는 최상위가 아니라 하위 모듈이므로 실제로는 핀이 아니라
#   다른 로직에 연결된다. 여기서는 주기의 20% 를 앞뒤 로직 예산으로 잡는다.
#   (예산을 크게 잡을수록 preprocessor 내부에 허용되는 시간이 줄어든다)
#------------------------------------------------------------------------------

set IO_BUDGET [expr {$CLK_PERIOD * 0.20}]

set_input_delay  -clock clk -max $IO_BUDGET $data_in
set_input_delay  -clock clk -min 0.100      $data_in

set_output_delay -clock clk -max $IO_BUDGET $all_out
set_output_delay -clock clk -min 0.100      $all_out


#------------------------------------------------------------------------------
# 4. 리셋
#
#   preprocessor 의 rst_n 은 always_ff 안에서 판정되는 동기 리셋이므로
#   별도 false path 가 필요 없다. 비동기로 바꿀 경우에만 아래를 켠다.
#------------------------------------------------------------------------------

# set_false_path -from [get_ports rst_n]


#------------------------------------------------------------------------------
# 5. [선택] 내부 레지스터 간 경로만 보고 싶을 때
#
#   "preprocessor 자체가 몇 MHz 까지 도는가" 만 알고 싶으면
#   위 3번 블록을 주석 처리하고 아래 두 줄을 살린다.
#   I/O 경로가 분석에서 빠지고 reg -> reg 경로만 남는다.
#------------------------------------------------------------------------------

# set_false_path -from $data_in
# set_false_path -to   $all_out
