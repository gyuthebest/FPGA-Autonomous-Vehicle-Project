`timescale 1ns / 1ps

import types_pkg::*;

// PL 판단 과정 클럭 단위 추적 testbench.
//
// tb_carla_axi_replay와 동일하게 REG0..REG9 이미지를 AXI4-Lite로 써 넣지만,
// 목적이 다르다.  이 testbench는 합/불을 판정하지 않고 **매 클럭마다** 판단
// 파이프라인의 모든 중간 신호를 CSV 한 줄로 기록한다.  보드가 없어도 PL이
// 왜 그런 결론을 냈는지 단계별로 되짚을 수 있다.
//
// 기록 단계:
//   S0  AXI 커밋      : sample_seq, valid
//   S1  preprocessor  : delta_*(1차 차분), pred_*(동역학 기준값)
//   S2  sensor_checker: 채널별 range/jump/stuck/noise/timeout 확정 비트
//   S2b consistency   : 관계식별 확정 비트
//   S3  reliability   : 채널별 NORMAL/DEGRADED/INVALID
//   S4  risk_types    : 원시 위험도 tier
//   S5  risk_control  : 유효 tier, 제동 요청/상한/최종, TD/MRM/HUD
//
// 출력:
//   pl_trace.csv  - 매 클럭 1행
//   pl_trace.vcd  - 파형 (+TRACE_VCD 플러스아규먼트)
//
// 실행: run_pl_trace.bat  (기본 입력 fixtures/pl_vectors_smoke.csv)

module tb_pl_trace;

    localparam integer AXI_ADDR_W = 6;
    localparam integer SIM_CLK_FREQ_HZ = 2000;
    localparam integer SAMPLE_RATE_HZ = 20;
    localparam integer NOMINAL_GAP_CYCLES = SIM_CLK_FREQ_HZ / SAMPLE_RATE_HZ;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n;
    logic [AXI_ADDR_W-1:0] awaddr, araddr;
    logic [2:0] awprot, arprot;
    logic awvalid, awready;
    logic [31:0] wdata;
    logic [3:0] wstrb;
    logic wvalid, wready;
    logic [1:0] bresp;
    logic bvalid, bready;
    logic arvalid, arready;
    logic [31:0] rdata;
    logic [1:0] rresp;
    logic rvalid, rready;

    longint unsigned cycle_count = 0;
    integer trace_fd;
    integer replay_count = 0;
    string  vector_path;
    string  trace_path;

    always @(posedge clk) cycle_count <= cycle_count + 1;

    top_controller #(
        .C_S_AXI_DATA_WIDTH(32),
        .C_S_AXI_ADDR_WIDTH(AXI_ADDR_W),
        .CLK_FREQ_HZ(SIM_CLK_FREQ_HZ),
        .SAMPLE_RATE_HZ(SAMPLE_RATE_HZ)
    ) dut (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(rst_n),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWPROT(awprot),
        .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(wstrb),
        .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARPROT(arprot),
        .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp),
        .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready)
    );

    //======================================================================
    // AXI4-Lite write
    //======================================================================
    task automatic axi_write(input logic [AXI_ADDR_W-1:0] addr,
                             input logic [31:0] data);
        @(negedge clk);
        awaddr = addr; awprot = 3'b000; awvalid = 1'b1;
        wdata = data; wstrb = 4'hF; wvalid = 1'b1; bready = 1'b1;
        do @(posedge clk); while (!(awready && wready));
        @(negedge clk);
        awvalid = 1'b0; wvalid = 1'b0;
        do @(posedge clk); while (!bvalid);
        @(negedge clk);
        bready = 1'b0;
    endtask

    // REG0..REG8을 먼저 쓰고 REG9(sample_seq)를 마지막에 쓴다. 실제 PS 순서다.
    task automatic commit_sample(input logic [31:0] words [10]);
        for (int i = 0; i < 9; i++) axi_write(i[AXI_ADDR_W-1:0] * 4, words[i]);
        axi_write(6'h24, words[9]);
        replay_count++;
    endtask

    //======================================================================
    // 클럭 단위 추적
    //======================================================================
    function automatic string csv_header();
        return {
          "cycle,sim_ns,rst_n,",
          // S0 AXI 커밋
          "s0_sample_seq_axi,s0_seq_pre,",
          // S1 preprocessor
          "s1_valid_s1,s1_distance,s1_approach_speed,s1_gyro_z,s1_accel_z,",
          "s1_temperature,s1_humidity,s1_lux,",
          "s1_delta_distance,s1_delta_gyro_z,s1_delta_accel_z,s1_delta_temp,",
          "s1_pred_gyro_z_1,s1_pred_distance,s1_gyro_z_x_S_GYR,",
          // 관계식 17(조향-yaw) / 관계식 3(accel_x 동역학) 의 기준값.
          // 지금까지 트레이스에 없어 모델과 한 번도 대조되지 않던 항목이다.
          "s1_pred_gyro_z_3,s1_pred_accel_x_1,s1_accel_x,s1_delta_accel_x,",
          // S2 개별 검사기 확정 비트 (채널 비트맵)
          "s2_range_err,s2_jump_err,s2_stuck_err,s2_noise_err,s2_timeout_err,",
          "s2_timeout_phase_cnt,",
          // S2b consistency
          "s2b_cons_gyro_z,s2b_cons_accel_x,s2b_cons_distance,",
          // S3 reliability 상태
          "s3_rel_distance,s3_rel_gyro_z,s3_rel_accel_z,s3_rel_temp,s3_rel_hum,",
          // S4 risk_types 원시 분류(risk_out_s1)와 신뢰도 보정 후 유효 위험도
          "s4_collision,s4_road_A,s4_road_B,s4_vision_A,s4_posture_C,",
          "s4e_collision,s4e_road_A,s4e_road_B,s4e_vision_A,s4e_posture_C,",
          // S5 risk_control
          "s5_eff_road_A,s5_eff_posture_C,s5_col_brake,s5_road_B_brake,",
          "s5_surface_cap,s5_lateral_cap,s5_brake_cap,s5_requested_brake,",
          "s5_final_brake,s5_final_accel,",
          "s5_td,s5_mrm,s5_hud,s5_td_remain,s5_valid_out"
        };
    endfunction

    always @(posedge clk) begin
        if (trace_fd != 0) begin
            $fwrite(trace_fd,
              "%0d,%0t,%0d,", cycle_count, $time, rst_n);

            // S0
            $fwrite(trace_fd, "%0d,%0d,",
              dut.sample_seq_axi, dut.u_preprocessor.sample_seq_in);

            // S1 preprocessor : 입력 스냅샷과 1차 차분, 동역학 기준값
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,",
              dut.u_preprocessor.valid_s1,
              $signed({1'b0, dut.u_preprocessor.sensor_data_in.distance}),
              $signed(dut.u_preprocessor.sensor_data_in.approach_speed),
              $signed(dut.u_preprocessor.sensor_data_in.gyro_z),
              $signed(dut.u_preprocessor.sensor_data_in.accel_z),
              $signed(dut.u_preprocessor.sensor_data_in.temperature),
              $signed({1'b0, dut.u_preprocessor.sensor_data_in.humidity}),
              $signed({1'b0, dut.u_preprocessor.sensor_data_in.lux}));

            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,",
              $signed(dut.u_preprocessor.processed_data_out.delta_distance),
              $signed(dut.u_preprocessor.processed_data_out.delta_gyro_z),
              $signed(dut.u_preprocessor.processed_data_out.delta_accel_z),
              $signed(dut.u_preprocessor.processed_data_out.delta_temp));

            // gyro consistency 양변을 나란히 남긴다.  좌변이 우변을 따라가면
            // Q-format이 맞는 것이고, 어느 한쪽이 갑자기 부호가 뒤집히면 wrap이다.
            $fwrite(trace_fd, "%0d,%0d,%0d,",
              $signed(dut.u_preprocessor.pred_data_out.pred_gyro_z_1),
              $signed(dut.u_preprocessor.pred_data_out.pred_distance),
              $signed(dut.u_preprocessor.sensor_data_in.gyro_z) * 1024);

            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,",
              $signed(dut.u_preprocessor.pred_data_out.pred_gyro_z_3),
              $signed(dut.u_preprocessor.pred_data_out.pred_accel_x_1),
              $signed(dut.u_preprocessor.sensor_data_in.accel_x),
              $signed(dut.u_preprocessor.processed_data_out.delta_accel_x));

            // S2 검사기 확정 비트
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,%0d,",
              dut.u_sensor_reliability.range_err,
              dut.u_sensor_reliability.jump_err,
              dut.u_sensor_reliability.stuck_err,
              dut.u_sensor_reliability.noise_err,
              dut.u_sensor_reliability.timeout_err,
              dut.u_sensor_reliability.timeout_phase_cnt);

            // S2b consistency
            $fwrite(trace_fd, "%0d,%0d,%0d,",
              dut.u_sensor_reliability.cons_err_gyro_z,
              dut.u_sensor_reliability.cons_err_accel_x,
              dut.u_sensor_reliability.cons_err_distance);

            // S3 reliability 상태 (0=NORMAL 1=DEGRADED 2=INVALID)
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,",
              dut.rel_out.distance.state, dut.rel_out.gyro_z.state,
              dut.rel_out.accel_z.state, dut.rel_out.temperature.state,
              dut.rel_out.humidity.state);

            // S4 risk_types 원시 분류
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,",
              dut.risk_out_s1.Ri_collision, dut.risk_out_s1.Ri_road_A,
              dut.risk_out_s1.Ri_road_B, dut.risk_out_s1.Ri_vision_A,
              dut.risk_out_s1.Ri_posture_C);

            // S4e 신뢰도 보정 후 유효 위험도 (risk_control 출력)
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,",
              dut.risk_out.Ri_collision, dut.risk_out.Ri_road_A,
              dut.risk_out.Ri_road_B, dut.risk_out.Ri_vision_A,
              dut.risk_out.Ri_posture_C);

            // S5 risk_control 제동 중재
            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,",
              dut.u_risk_control.eff_tier_road_A,
              dut.u_risk_control.eff_tier_posture_C,
              dut.u_risk_control.col_brake,
              dut.u_risk_control.road_B_brake,
              dut.u_risk_control.surface_brake_cap,
              dut.u_risk_control.lateral_brake_cap,
              dut.u_risk_control.brake_cap,
              dut.u_risk_control.requested_brake,
              dut.u_risk_control.final_brake,
              dut.u_risk_control.final_accelerator);

            $fwrite(trace_fd, "%0d,%0d,%0d,%0d,%0d\n",
              dut.transition_demand, dut.mrm, dut.hud_warning,
              dut.td_remain_sec, dut.valid_out_rel_risk);
        end
    end

    //======================================================================
    // 벡터 재생
    //======================================================================
    integer vec_fd;
    string  line;
    logic [31:0] words [10];

    initial begin
        awaddr = '0; awprot = '0; awvalid = 1'b0;
        wdata = '0; wstrb = '0; wvalid = 1'b0; bready = 1'b0;
        araddr = '0; arprot = '0; arvalid = 1'b0; rready = 1'b0;
        rst_n = 1'b0;

        // 파일명은 고정한다.  run_pl_trace.bat이 선택한 벡터를 vectors.csv로
        // 복사해 넣는다.  (이 Vivado 버전의 xsim은 -testplusarg를 스냅샷에
        // 전달하지 못해 plusargs를 쓰지 않는다.)
        vector_path = "vectors.csv";
        trace_path  = "pl_trace.csv";

        trace_fd = $fopen(trace_path, "w");
        if (trace_fd == 0) begin
            $display("ERROR: cannot open trace file %s", trace_path);
            $finish;
        end
        $fwrite(trace_fd, "%s\n", csv_header());

`ifdef TRACE_VCD
        // 파형은 컴파일 시 -d TRACE_VCD 로만 켠다. 전체 계층을 덤프하므로
        // 긴 캡처에서는 파일이 매우 커진다.
        $dumpfile("pl_trace.vcd");
        $dumpvars(0, tb_pl_trace);
`endif

        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (20) @(posedge clk);

        vec_fd = $fopen(vector_path, "r");
        if (vec_fd == 0) begin
            $display("ERROR: cannot open vector file %s", vector_path);
            $fclose(trace_fd);
            $finish;
        end

        void'($fgets(line, vec_fd));      // 헤더 폐기

        while (!$feof(vec_fd)) begin
            int unsigned seq;
            longint unsigned gap_ns;
            int code;
            if ($fgets(line, vec_fd) == 0) break;
            // 형식: sample_seq, host_gap_ns, reg0..reg9 (reg는 16진수)
            code = $sscanf(line, "%d,%d,%h,%h,%h,%h,%h,%h,%h,%h,%h,%h",
                     seq, gap_ns, words[0], words[1], words[2], words[3],
                     words[4], words[5], words[6], words[7], words[8], words[9]);
            if (code != 12) continue;
            commit_sample(words);
            repeat (NOMINAL_GAP_CYCLES) @(posedge clk);
        end
        $fclose(vec_fd);

        repeat (100) @(posedge clk);

        $display("============================================================");
        $display("PL TRACE: %0d samples replayed, %0d clock rows -> %s",
                 replay_count, cycle_count, trace_path);
        $display("============================================================");
        $fclose(trace_fd);
        trace_fd = 0;
        $finish;
    end

endmodule
