`timescale 1ns / 1ps

import types_pkg::*;

// Self-checking verification environment written from scratch.
// It combines unit-level boundary tests with a real AXI4-Lite top-level test.
module tb_pl_full_verification;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    integer pass_count = 0;
    integer fail_count = 0;

    task automatic check_eq(
        input string name,
        input logic [63:0] actual,
        input logic [63:0] expected
    );
        if (actual === expected) begin
            pass_count++;
            $display("[PASS] %s actual=0x%0h", name, actual);
        end else begin
            fail_count++;
            $display("[FAIL] %s actual=0x%0h expected=0x%0h", name, actual, expected);
        end
    endtask

    task automatic check_true(input string name, input logic condition);
        check_eq(name, {63'd0, condition}, 64'd1);
    endtask

    //--------------------------------------------------------------------------
    // Preprocessor DUT
    //--------------------------------------------------------------------------
    logic pre_rst_n;
    sensor_data_t pre_sensor_in, pre_sensor_out;
    sim_data_t pre_sim_in, pre_sim_out;
    processed_data_t pre_processed, pre_prev_processed;
    pred_data_t pre_pred;
    logic [31:0] pre_seq_in, pre_seq_out, pre_clk_cnt;
    logic pre_valid;
    logic pre_mask1, pre_mask2, pre_mask3;

    preprocessor u_preprocessor_unit (
        .clk(clk), .rst_n(pre_rst_n),
        .sensor_data_in(pre_sensor_in), .sim_data_in(pre_sim_in),
        .sample_seq_in(pre_seq_in),
        .timeout_mask_1s(1'b0),
        .consistency_mask_1s_approach_speed(1'b0),
        .consistency_mask_20s_approach_speed(1'b0),
        .sensor_data_out(pre_sensor_out), .sim_data_out(pre_sim_out),
        .processed_data_out(pre_processed),
        .prev_processed_data_out(pre_prev_processed),
        .pred_data_out(pre_pred), .valid_s1(pre_valid),
        .sample_seq_out(pre_seq_out), .clk_cnt(pre_clk_cnt),
        .consistency_mask_1(pre_mask1),
        .consistency_mask_2(pre_mask2),
        .consistency_mask_3(pre_mask3)
    );

    task automatic pre_commit(input logic [31:0] seq);
        @(negedge clk);
        pre_seq_in = seq;
        @(posedge clk);
        #1;
    endtask

    //--------------------------------------------------------------------------
    // Generic sensor checker DUT. Small parameters make every mechanism testable.
    //--------------------------------------------------------------------------
    logic chk_rst_n, chk_valid, chk_stuck_mask;
    logic [31:0] chk_clk_cnt;
    logic signed [15:0] chk_sensor;
    logic signed [16:0] chk_delta, chk_prev_delta, chk_trigger1, chk_trigger2;
    logic [2:0] chk_situation;
    logic chk_range, chk_jump, chk_stuck, chk_timeout, chk_noise;
    logic chk_timeout_mask1, chk_timeout_mask2;

    each_sensor_check #(
        .WIDTH(16), .DW(17), .TW(17),
        .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(1),
        .UPDATE_CLK_X2(8), .DROP_N(2), .HISTORY(4),
        .RANGE_THRESHOLD_MIN(0), .RANGE_THRESHOLD_MAX(100),
        .JUMP_THRESHOLD(10), .STUCK_THRESHOLD(0),
        .NOISE_THRESHOLD_1(5), .NOISE_THRESHOLD_2(2),
        .RANGE_U(1), .RANGE_D(1), .RANGE_N(1),
        .JUMP_U(1), .JUMP_D(1), .JUMP_N(2),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(3),
        .TIMEOUT_U(1), .TIMEOUT_D(1), .TIMEOUT_N(1)
    ) u_sensor_checker_unit (
        .clk(clk), .rst_n(chk_rst_n), .valid_s1(chk_valid),
        .clk_cnt(chk_clk_cnt), .stuck_mask(chk_stuck_mask),
        .sensor_data(chk_sensor), .processed_data(chk_delta),
        .prev_processed_data(chk_prev_delta),
        .trig_val_1(chk_trigger1), .trig_val_2(chk_trigger2),
        .situation(chk_situation),
        .range_error(chk_range), .jump_error(chk_jump),
        .stuck_error(chk_stuck), .timeout_error(chk_timeout),
        .noise_error(chk_noise),
        .timeout_mask_1s(chk_timeout_mask1),
        .timeout_mask_2s(chk_timeout_mask2)
    );

    task automatic chk_reset;
        chk_valid = 0;
        chk_stuck_mask = 0;
        chk_clk_cnt = 0;
        chk_sensor = 50;
        chk_delta = 0;
        chk_prev_delta = 0;
        chk_trigger1 = 0;
        chk_trigger2 = 0;
        chk_situation = 0;
        chk_rst_n = 0;
        repeat (2) @(posedge clk);
        chk_rst_n = 1;
        @(negedge clk);
    endtask

    task automatic chk_sample;
        chk_valid = 1;
        @(posedge clk);
        #1;
        @(negedge clk);
        chk_valid = 0;
    endtask

    //--------------------------------------------------------------------------
    // Consistency checker and 20-second mask DUTs
    //--------------------------------------------------------------------------
    logic cons_rst_n, cons_valid, cons_timeout_mask, cons_mask;
    logic signed [15:0] cons_sensor, cons_pred;
    logic cons_error;

    consistency_check #(
        .WIDTH(16), .S(1), .CONSISTENCY_THRESHOLD(5),
        .CONSISTENCY_U(1), .CONSISTENCY_D(1), .CONSISTENCY_N(2)
    ) u_consistency_unit (
        .clk(clk), .rst_n(cons_rst_n), .valid_s1(cons_valid),
        .timeout_mask_2s(cons_timeout_mask), .consistency_mask(cons_mask),
        .sensor_data(cons_sensor), .pred_data(cons_pred),
        .consistency_error(cons_error)
    );

    logic mask_rst_n, mask_stuck_in, mask_cons_in;
    logic mask_stuck_out, mask_cons_out;
    mask_20s #(.CLK_FREQ(2)) u_mask_unit (
        .clk(clk), .rst_n(mask_rst_n),
        .stuck_err(mask_stuck_in),
        .consistency_error_approach_speed(mask_cons_in),
        .stuck_mask_20s(mask_stuck_out),
        .consistency_mask_20s_approach_speed(mask_cons_out)
    );

    task automatic cons_sample;
        cons_valid = 1;
        @(posedge clk);
        #1;
        @(negedge clk);
        cons_valid = 0;
    endtask

    //--------------------------------------------------------------------------
    // Risk classification DUT
    //--------------------------------------------------------------------------
    logic rt_rst_n, rt_valid, rt_valid_out;
    logic [31:0] rt_seq_in, rt_seq_out;
    sensor_data_t rt_sensor_in, rt_sensor_out;
    sim_data_t rt_sim_in, rt_sim_out;
    risk_t rt_risk;

    risk_types u_risk_types_unit (
        .clk(clk), .rst_n(rt_rst_n), .sample_seq_in(rt_seq_in),
        .valid_in(rt_valid), .sensor_data_in(rt_sensor_in),
        .sim_data_in(rt_sim_in), .sensor_data_out(rt_sensor_out),
        .sim_data_out(rt_sim_out), .sample_seq_out(rt_seq_out),
        .valid_out(rt_valid_out), .risk_out(rt_risk)
    );

    task automatic rt_sample;
        @(negedge clk);
        rt_seq_in++;
        rt_valid = 1;
        @(posedge clk);
        #1;
        @(negedge clk);
        rt_valid = 0;
    endtask

    //--------------------------------------------------------------------------
    // Risk-control DUT
    //--------------------------------------------------------------------------
    logic rc_rst_n, rc_valid_risk, rc_valid_rel;
    logic [31:0] rc_seq_risk_in, rc_seq_rel_in;
    sensor_data_t rc_sensor_in, rc_sensor_out;
    sim_data_t rc_sim_in, rc_sim_out;
    reliability_state_t rc_rel_in, rc_rel_out;
    risk_t rc_risk_in, rc_risk_out;
    logic [31:0] rc_seq_risk_out, rc_seq_rel_out;
    logic [1:0] rc_valid_out;
    logic rc_td, rc_hud, rc_mrm;
    logic [3:0] rc_td_sec;

    risk_control_2 #(.CLK_FREQ(20)) u_risk_control_unit (
        .clk(clk), .rst_n(rc_rst_n),
        .valid_in(rc_valid_risk), .valid_in_rel(rc_valid_rel),
        .sample_seq_in(rc_seq_risk_in), .sample_seq_in_rel(rc_seq_rel_in),
        .sensor_data_in(rc_sensor_in), .sim_data_in(rc_sim_in),
        .rel_in(rc_rel_in), .risk_in(rc_risk_in),
        .sample_seq_out_risk(rc_seq_risk_out),
        .sample_seq_out_rel(rc_seq_rel_out),
        .valid_out_rel_risk(rc_valid_out), .risk_out(rc_risk_out),
        .rel_out(rc_rel_out), .sensor_data_out(rc_sensor_out),
        .sim_data_out(rc_sim_out), .transition_demand(rc_td),
        .hud_warning(rc_hud), .mrm(rc_mrm), .td_remain_sec(rc_td_sec)
    );

    task automatic rc_sample;
        @(negedge clk);
        rc_seq_risk_in++;
        rc_seq_rel_in = rc_seq_risk_in;
        rc_valid_risk = 1;
        rc_valid_rel = 1;
        @(posedge clk);
        #1;
        @(negedge clk);
        rc_valid_risk = 0;
        rc_valid_rel = 0;
    endtask

    task automatic rc_reset;
        rc_valid_risk = 0;
        rc_valid_rel = 0;
        rc_seq_risk_in = 0;
        rc_seq_rel_in = 0;
        rc_sensor_in = '0;
        rc_sim_in = '0;
        rc_rel_in = '0;
        rc_risk_in = '0;
        rc_rst_n = 0;
        repeat (3) @(posedge clk);
        rc_rst_n = 1;
        @(negedge clk);
    endtask

    //--------------------------------------------------------------------------
    // Complete top_controller over its real AXI4-Lite port
    //--------------------------------------------------------------------------
    localparam int AXI_ADDR_W = 6;
    logic top_rst_n;
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

    top_controller #(
        .C_S_AXI_DATA_WIDTH(32), .C_S_AXI_ADDR_WIDTH(AXI_ADDR_W),
        .CLK_FREQ_HZ(200), .SAMPLE_RATE_HZ(10)
    ) u_top (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(top_rst_n),
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

    task automatic axi_write(
        input logic [AXI_ADDR_W-1:0] addr,
        input logic [31:0] data,
        input logic [3:0] strb
    );
        integer guard;
        @(negedge clk);
        awaddr = addr;
        awvalid = 1;
        wdata = data;
        wstrb = strb;
        wvalid = 1;
        guard = 0;
        while (!(awready && wready) && guard < 20) begin
            @(posedge clk);
            #1;
            guard++;
        end
        if (guard >= 20) begin
            fail_count++;
            $display("[FAIL] AXI write handshake timeout addr=0x%0h", addr);
        end
        // READY is registered in the Xilinx template. Keep VALID asserted
        // through the following rising edge where the transfer is sampled.
        @(posedge clk);
        #1;
        @(negedge clk);
        awvalid = 0;
        wvalid = 0;
        bready = 1;
        guard = 0;
        while (!bvalid && guard < 20) begin
            @(posedge clk);
            #1;
            guard++;
        end
        if (guard >= 20 || bresp !== 2'b00) begin
            fail_count++;
            $display("[FAIL] AXI write response addr=0x%0h bresp=%b", addr, bresp);
        end
        @(negedge clk);
        bready = 0;
    endtask

    task automatic axi_read(
        input logic [AXI_ADDR_W-1:0] addr,
        output logic [31:0] data
    );
        integer guard;
        @(negedge clk);
        araddr = addr;
        arvalid = 1;
        guard = 0;
        while (!arready && guard < 20) begin
            @(posedge clk);
            #1;
            guard++;
        end
        // As on the write channel, hold VALID through the sampling edge.
        @(posedge clk);
        #1;
        @(negedge clk);
        arvalid = 0;
        rready = 1;
        guard = 0;
        while (!rvalid && guard < 20) begin
            @(posedge clk);
            #1;
            guard++;
        end
        data = rdata;
        if (guard >= 20 || rresp !== 2'b00) begin
            fail_count++;
            $display("[FAIL] AXI read response addr=0x%0h rresp=%b", addr, rresp);
        end
        @(negedge clk);
        rready = 0;
    endtask

    task automatic top_wait_seq(input logic [31:0] seq);
        integer guard;
        guard = 0;
        while ((u_top.sample_seq_risk !== seq || u_top.sample_seq_rel !== seq) && guard < 80) begin
            @(posedge clk);
            #1;
            guard++;
        end
        if (guard >= 80) begin
            fail_count++;
            $display("[FAIL] Top pipeline sequence timeout expected=%0d risk=%0d rel=%0d",
                     seq, u_top.sample_seq_risk, u_top.sample_seq_rel);
        end
    endtask

    task automatic top_commit(input logic [31:0] seq);
        axi_write(6'h24, seq, 4'hF);
        top_wait_seq(seq);
    endtask

    task automatic init_all;
        pre_rst_n = 0;
        pre_sensor_in = '0;
        pre_sim_in = '0;
        pre_seq_in = 0;
        chk_rst_n = 0;
        chk_valid = 0;
        chk_stuck_mask = 0;
        chk_clk_cnt = 0;
        chk_sensor = 0;
        chk_delta = 0;
        chk_prev_delta = 0;
        chk_trigger1 = 0;
        chk_trigger2 = 0;
        chk_situation = 0;
        cons_rst_n = 0;
        cons_valid = 0;
        cons_timeout_mask = 0;
        cons_mask = 0;
        cons_sensor = 0;
        cons_pred = 0;
        mask_rst_n = 0;
        mask_stuck_in = 0;
        mask_cons_in = 0;
        rt_rst_n = 0;
        rt_valid = 0;
        rt_seq_in = 0;
        rt_sensor_in = '0;
        rt_sim_in = '0;
        rc_rst_n = 0;
        rc_valid_risk = 0;
        rc_valid_rel = 0;
        rc_seq_risk_in = 0;
        rc_seq_rel_in = 0;
        rc_sensor_in = '0;
        rc_sim_in = '0;
        rc_rel_in = '0;
        rc_risk_in = '0;
        top_rst_n = 0;
        awaddr = 0;
        awprot = 0;
        awvalid = 0;
        wdata = 0;
        wstrb = 0;
        wvalid = 0;
        bready = 0;
        araddr = 0;
        arprot = 0;
        arvalid = 0;
        rready = 0;
    endtask

    logic [31:0] rd;

    initial begin
        init_all();
        repeat (4) @(posedge clk);

        //------------------------------------------------------------------
        $display("\n=== 1. PREPROCESSOR / VALID / DELTA / PREDICTOR ===");
        pre_sensor_in.distance = 15'd100;
        pre_sensor_in.approach_speed = 13'sd10;
        pre_sim_in.situation = 3'b001;
        pre_rst_n = 1;
        pre_commit(1);
        check_true("pre.valid on new sequence", pre_valid);
        check_eq("pre.sequence", pre_seq_out, 1);
        check_eq("pre.distance pass-through", pre_sensor_out.distance, 100);
        check_eq("pre.initial distance delta", pre_processed.delta_distance, 100);
        check_eq("pre.predictor reset value", pre_pred.pred_distance, 4000);

        @(posedge clk); #1;
        check_eq("pre.valid low without new sequence", pre_valid, 0);
        check_true("pre.clock counter increments", pre_clk_cnt > 0);

        pre_sensor_in.distance = 15'd130;
        pre_sensor_in.approach_speed = 13'sd20;
        pre_sim_in.situation = 3'b000;
        pre_commit(2);
        check_eq("pre.current distance delta", pre_processed.delta_distance, 30);
        check_eq("pre.previous distance delta", pre_prev_processed.delta_distance, 100);
        check_eq("pre.linear distance prediction", pre_pred.pred_distance, 3970);

        //------------------------------------------------------------------
        $display("\n=== 2. RANGE / JUMP / STUCK / TIMEOUT / NOISE ===");
        chk_reset();
        chk_sensor = 101;
        chk_sample();
        check_true("range immediate threshold", chk_range);
        repeat (20) chk_sample();
        check_eq("range counter saturates at threshold",
                 u_sensor_checker_unit.range_cnt, 1);
        chk_sensor = 50;
        chk_sample();
        check_eq("range clears after bounded recovery", chk_range, 0);

        chk_reset();
        chk_sensor = 50;
        chk_delta = 20;
        chk_prev_delta = 0;
        chk_sample();
        check_eq("jump debounce first hit", chk_jump, 0);
        chk_delta = -20;
        chk_prev_delta = 20;
        chk_sample();
        check_true("jump debounce confirmation", chk_jump);
        repeat (20) begin
            chk_delta = -20;
            chk_prev_delta = 20;
            chk_sample();
        end
        check_eq("jump counter saturates at threshold",
                 u_sensor_checker_unit.jump_cnt, 2);
        chk_delta = 0;
        chk_prev_delta = 0;
        repeat (2) chk_sample();
        check_eq("jump clears after bounded recovery", chk_jump, 0);

        chk_reset();
        chk_sensor = 50;
        chk_delta = 0;
        chk_prev_delta = 0;
        chk_trigger1 = 1;
        repeat (3) chk_sample();
        check_true("stuck trigger and debounce", chk_stuck);
        repeat (20) chk_sample();
        check_eq("stuck counter saturates at threshold",
                 u_sensor_checker_unit.stuck_cnt, 3);
        chk_delta = 1;
        repeat (3) chk_sample();
        check_eq("stuck clears after bounded recovery", chk_stuck, 0);

        chk_reset();
        chk_valid = 0;
        chk_clk_cnt = 7;
        @(posedge clk); #1;
        check_true("timeout threshold", chk_timeout);
        check_true("timeout recovery mask armed", chk_timeout_mask2);

        chk_reset();
        chk_sensor = 50;
        chk_trigger1 = 0;
        chk_delta = 10; chk_prev_delta = -10; chk_sample();
        chk_delta = -10; chk_prev_delta = 10; chk_sample();
        chk_delta = 10; chk_prev_delta = -10; chk_sample();
        chk_delta = -10; chk_prev_delta = 10; chk_sample();
        check_true("noise MAD/sign-flip window", chk_noise);

        //------------------------------------------------------------------
        $display("\n=== 3. CONSISTENCY AND 20-SECOND MASK ===");
        cons_sensor = 100;
        cons_pred = 100;
        cons_timeout_mask = 0;
        cons_mask = 0;
        cons_rst_n = 1;
        cons_sample();
        check_eq("consistency normal", cons_error, 0);
        cons_pred = 80;
        repeat (2) cons_sample();
        check_true("consistency debounce confirmation", cons_error);
        repeat (20) cons_sample();
        check_eq("consistency counter saturates at threshold",
                 u_consistency_unit.consistency_cnt, 2);
        cons_pred = 100;
        repeat (2) cons_sample();
        check_eq("consistency clears after bounded recovery", cons_error, 0);

        cons_rst_n = 0;
        repeat (2) @(posedge clk);
        cons_rst_n = 1;
        cons_mask = 1;
        cons_pred = 80;
        repeat (3) cons_sample();
        check_eq("consistency mask freezes counter", cons_error, 0);

        mask_rst_n = 1;
        @(negedge clk);
        mask_stuck_in = 1;
        mask_cons_in = 1;
        @(posedge clk); #1;
        mask_stuck_in = 0;
        mask_cons_in = 0;
        check_true("20s stuck mask asserted", mask_stuck_out);
        check_true("20s consistency mask asserted", mask_cons_out);
        repeat (40) @(posedge clk);
        #1;
        check_eq("20s stuck mask clears", mask_stuck_out, 0);
        check_eq("20s consistency mask clears", mask_cons_out, 0);

        //------------------------------------------------------------------
        $display("\n=== 4. ALL EIGHT RISK CLASSIFIERS ===");
        rt_sensor_in = '0;
        rt_sim_in = '0;
        rt_sensor_in.distance = 1000;
        rt_sensor_in.approach_speed = 100;
        rt_sensor_in.temperature = 20;
        rt_sensor_in.humidity = 50;
        rt_sensor_in.lux = 20000;
        rt_sensor_in.accel_z = 980;
        rt_rst_n = 1;
        rt_sample();
        check_eq("risk collision safe", rt_risk.Ri_collision, 0);
        check_eq("risk road surface dry", rt_risk.Ri_road_A, 0);
        check_eq("risk road impact normal", rt_risk.Ri_road_B, 0);
        check_eq("risk vision lux normal", rt_risk.Ri_vision_A, 0);
        check_eq("risk weather normal", rt_risk.Ri_vision_B, 0);
        check_eq("risk roll normal", rt_risk.Ri_posture_A, 0);
        check_eq("risk yaw normal", rt_risk.Ri_posture_B, 0);
        check_eq("risk lateral normal", rt_risk.Ri_posture_C, 0);

        // Emergency 경계가 TTC 1.5초에서 **1.4초**로 바뀌었다.
        // 이전 벡터 distance=150 / approach=100 은 TTC 정확히 1.5초라
        // 새 사양에서는 Critical 이다 (아래에 경계 시험으로 따로 둔다).
        // 여기서는 확실히 Emergency 인 TTC 1.3초를 쓴다.
        rt_sensor_in.distance = 130;
        rt_sensor_in.approach_speed = 100;
        rt_sensor_in.temperature = -50;
        rt_sensor_in.humidity = 90;
        rt_sensor_in.accel_z = -1000;
        rt_sim_in.speed_x = 1000;
        rt_sensor_in.lux = 49;
        rt_sim_in.weather = 2;
        rt_sensor_in.gyro_x = 698;
        rt_sensor_in.gyro_z = 1047;
        rt_sensor_in.accel_y = 784;
        rt_sample();
        check_eq("risk collision emergency", rt_risk.Ri_collision, 4);
        check_eq("risk road black ice", rt_risk.Ri_road_A, 3);
        check_eq("risk road severe impact", rt_risk.Ri_road_B, 3);
        check_eq("risk vision darkness", rt_risk.Ri_vision_A, 3);
        check_eq("risk weather propagation", rt_risk.Ri_vision_B, 2);
        check_eq("risk roll danger", rt_risk.Ri_posture_A, 1);
        check_eq("risk yaw danger", rt_risk.Ri_posture_B, 2);
        check_eq("risk lateral danger", rt_risk.Ri_posture_C, 2);

        //------------------------------------------------------------------
        $display("\n=== 5. RISK FUSION / CONTROL / TD / MRM ===");
        rc_reset();
        rc_sim_in.accelerator = 10;
        rc_sim_in.brake = 0;
        rc_sim_in.speed_x = 1000;
        rc_sim_in.speed_limit = 3200;
        rc_sim_in.gear = 2;
        rc_sim_in.rpm = 2;
        rc_risk_in.Ri_collision = 4;
        rc_sample();
        check_eq("collision emergency accelerator", rc_sim_out.accelerator, 0);
        check_eq("collision emergency brake", rc_sim_out.brake, 10);
        check_true("collision emergency hazard", rc_sim_out.hazard);

        // --- 충돌 tier 경계 (사양: Emergency 는 TTC <= 1.4초) --------------
        // 근사식이 1.40625 이므로 approach=100 기준 distance 140 까지 Emergency,
        // 141 부터 Critical 이어야 한다.  이전 구현(1.5초)에서는 150 까지
        // Emergency 였다.
        rt_sensor_in.approach_speed = 100;
        rt_sensor_in.distance = 140;
        rt_sample();
        check_eq("collision boundary 1.40s emergency", rt_risk.Ri_collision, 4);

        rt_sensor_in.distance = 141;
        rt_sample();
        check_eq("collision boundary 1.41s critical", rt_risk.Ri_collision, 3);

        rt_sensor_in.distance = 150;
        rt_sample();
        check_eq("collision boundary 1.50s critical", rt_risk.Ri_collision, 3);

        // 최소 기어는 1단이다. gear=1 이면 RPM 조건을 만족해도 내리지 않는다.
        rc_reset();
        rc_sim_in.accelerator = 10;
        rc_sim_in.speed_x = 1000;
        rc_sim_in.speed_limit = 3200;
        rc_sim_in.gear = 1;
        rc_sim_in.rpm = 1;
        rc_risk_in.Ri_collision = 3;
        rc_sample();
        check_eq("collision downshift keeps min gear 1", rc_sim_out.gear, 1);

        rc_sim_in.gear = 2;
        rc_sample();
        check_eq("collision downshift from gear 2", rc_sim_out.gear, 1);

        rc_reset();
        rc_sim_in.accelerator = 10;
        rc_sim_in.brake = 3;
        rc_sim_in.speed_x = 0;
        rc_sim_in.speed_limit = 3200;
        rc_risk_in.Ri_road_A = 3;
        rc_sample();
        check_eq("black ice accelerator cap", rc_sim_out.accelerator, 4);
        // 마찰 비례 블렌딩: 요청 제동 3이 BLACK ICE 상한 3과 같아 그대로 통과한다.
        // 이전 정책은 저마찰에서 제동을 0으로 강제해, 동시에 발생한 충돌
        // EMERGENCY 요청까지 지워버렸다.
        check_eq("black ice brake blended not suppressed", rc_sim_out.brake, 3);
        check_eq("black ice speed limit 50 percent", rc_sim_out.speed_limit, 1600);

        rc_reset();
        rc_sim_in.accelerator = 10;
        rc_sim_in.speed_x = 0;
        rc_sim_in.speed_limit = 3200;
        rc_risk_in.Ri_vision_B = 2;
        rc_sample();
        check_eq("weather accelerator cap", rc_sim_out.accelerator, 8);
        check_eq("weather speed limit 70 percent", rc_sim_out.speed_limit, 2240);
        check_true("weather headlight", rc_sim_out.headlight);
        check_true("weather hazard", rc_sim_out.hazard);

        rc_reset();
        rc_sim_in.accelerator = 9;
        rc_sim_in.brake = 1;
        rc_sim_in.manual_mode = 1;
        rc_risk_in.Ri_vision_B = 2;
        rc_sample();
        check_eq("manual accelerator pass-through", rc_sim_out.accelerator, 9);
        check_eq("manual brake pass-through", rc_sim_out.brake, 1);
        check_true("manual still applies safety light", rc_sim_out.headlight);

        rc_reset();
        rc_sim_in.accelerator = 10;
        rc_sim_in.speed_limit = 3200;
        rc_rel_in.distance.state = 2'b01;
        rc_sample();
        check_eq("degraded collision raises tier", rc_risk_out.Ri_collision, 1);
        check_eq("degraded collision removes accelerator", rc_sim_out.accelerator, 0);

        rc_rel_in.distance.state = 2'b10;
        rc_sample();
        check_true("invalid reliability HUD", rc_hud);
        repeat (230) @(posedge clk);
        #1;
        check_eq("TD countdown reaches zero", rc_td_sec, 0);
        check_true("MRM asserted", rc_mrm);
        rc_sample();
        check_eq("MRM accelerator", rc_sim_out.accelerator, 0);
        check_eq("MRM brake", rc_sim_out.brake, 3);
        check_true("MRM hazard", rc_sim_out.hazard);
        rc_sim_in.manual_mode = 1;
        repeat (2) @(posedge clk);
        #1;
        check_eq("manual mode clears TD countdown", rc_td_sec, 11);

        //------------------------------------------------------------------
        $display("\n=== 6. COMPLETE AXI -> PIPELINE -> AXI INTEGRATION ===");
        top_rst_n = 0;
        repeat (4) @(posedge clk);
        top_rst_n = 1;

        // Baseline frame. REG9 is intentionally omitted first to verify commit.
        axi_write(6'h00, 32'h0000_0000, 4'hF);               // ay, ax
        axi_write(6'h04, {16'd0, 16'd980}, 4'hF);            // gx, az
        axi_write(6'h08, 32'h0000_0000, 4'hF);               // gz, gy
        axi_write(6'h0C, 32'h0000_0000, 4'hF);               // iy, ix
        axi_write(6'h10, 32'h0000_0000, 4'hF);               // sy, sx, iz
        axi_write(6'h14, {7'd50, 10'd12, 15'd1000}, 4'hF);   // hum, approach=96, dist
        axi_write(6'h18, {2'd0, 2'd0, 4'd10, 8'd0, 18'd20000}, 4'hF);
        axi_write(6'h1C, {2'd2, 2'd1, 4'd0, 5'd0, 8'd100, 11'd20}, 4'hF);
        axi_write(6'h20, 32'h0000_0000, 4'hF);
        repeat (4) @(posedge clk);
        check_eq("REG0..8 do not commit a frame", u_top.sample_seq_s1, 0);

        top_commit(1);
        axi_read(6'h2C, rd);
        check_eq("AXI risk sequence", rd, 1);
        axi_read(6'h30, rd);
        check_eq("AXI reliability sequence", rd, 1);
        axi_read(6'h34, rd);
        check_eq("AXI final accelerator", rd[12:9], 10);
        check_eq("AXI final brake", rd[16:13], 0);
        axi_read(6'h38, rd);
        check_eq("AXI final speed limit", rd[12:0], 3200);

        // Verify WSTRB by changing only the low byte of REG8.
        axi_write(6'h20, 32'hFFFF_FF01, 4'b0001);
        check_eq("AXI byte strobe low byte", u_top.u_axi_slave.slv_reg8, 1);

        // Project setting: RANGE_U=1, RANGE_D=1, RANGE_N=3.
        axi_write(6'h14, {7'd50, 10'd12, 15'd20001}, 4'hF);
        top_commit(2);
        check_eq("range debounce first invalid sample",
                 u_top.u_sensor_reliability.range_err[0], 0);

        top_commit(3);
        check_eq("range debounce second invalid sample",
                 u_top.u_sensor_reliability.range_err[0], 0);
        top_commit(4);
        check_true("range debounce third invalid sample",
                   u_top.u_sensor_reliability.range_err[0]);
        top_commit(5);
        axi_read(6'h28, rd);
        check_eq("AXI distance reliability eventually INVALID", rd[1:0], 2);

        // Timeout intent B: one evidence tick per UPDATE_CLK_X2 and TN=10.
        // The scaled top uses UPDATE_CLK_X2=40 cycles, so nine missing-sample
        // intervals must not confirm and the tenth must confirm.
        repeat (365) @(posedge clk);
        #1;
        check_eq("timeout not confirmed after nine intervals",
                 u_top.u_sensor_reliability.timeout_err[0], 0);
        repeat (45) @(posedge clk);
        #1;
        check_true("timeout confirmed after ten intervals",
                   u_top.u_sensor_reliability.timeout_err[0]);

        // Additional missing intervals must not grow the diagnostic counter
        // beyond TN.  The confirmed state is deliberately held for the first
        // two recovery samples so PS/CARLA can observe it after communication
        // resumes; subsequent valid samples finish bounded healing.
        repeat (400) @(posedge clk);
        #1;
        check_eq("timeout counter saturates at threshold",
                 u_top.u_sensor_reliability.u_chk_distance.timeout_cnt, 10);
        // Recover with the protocol's no-target sentinel.  Sentinel masking
        // may suppress distance plausibility diagnostics, but it must never
        // conceal loss of the complete PS-to-PL transport stream.
        axi_write(6'h14, {7'd50, 10'd0, 15'd20000}, 4'hF);
        top_commit(6);
        check_true("timeout remains visible on first recovery sample",
                   u_top.u_sensor_reliability.timeout_err[0]);
        axi_read(6'h28, rd);
        check_eq("no-target sentinel preserves distance timeout state",
                 rd[1:0], 2);
        top_commit(7);
        check_eq("timeout clears after recovery hold",
                 u_top.u_sensor_reliability.timeout_err[0], 0);
        top_commit(8);
        top_commit(9);
        top_commit(10);
        check_eq("timeout counter completes bounded healing",
                 u_top.u_sensor_reliability.u_chk_distance.timeout_cnt, 0);

        //------------------------------------------------------------------
        $display("\n============================================================");
        $display("FULL PL VERIFICATION RESULT: PASS=%0d FAIL=%0d", pass_count, fail_count);
        $display("============================================================");
        $finish;
    end

endmodule
