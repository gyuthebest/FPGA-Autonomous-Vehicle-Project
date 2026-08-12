`timescale 1ns / 1ps

import types_pkg::*;

module sensor_reliability (
    input clk,//
    input rst_n,//
    input valid_s1,//
    input [31:0] sample_seq_in,//
    input [31:0] clk_cnt, //
    input sensor_data_t sensor_data_in,//
    input processed_data_t processed_data_in,//
    input processed_data_t prev_processed_data_in,//
    input pred_data_t pred_data_in,//
    input consistency_mask_1,//
    input consistency_mask_2,//
    input consistency_mask_3,//
    input [2:0] situation, //
    output logic timeout_mask_1s,//
    output logic consistency_mask_1s_approach_speed,//
    output logic consistency_mask_20s_approach_speed, // 20s 추가//

    output logic [31:0] sample_seq_out_rel,//
    output logic valid_out_rel,//
    output reliability_state_t reliability_out//
);

    //==========================================================================
    // Channel Index  (sensor_data_t order)
    //==========================================================================

    localparam int NCH = 11;
    localparam int CH_DIST = 0,  CH_APSP = 1,
                   CH_AX   = 2,  CH_AY   = 3,  CH_AZ   = 4,
                   CH_GX   = 5,  CH_GY   = 6,  CH_GZ   = 7,
                   CH_TEMP = 8,  CH_HUM  = 9,  CH_LUX  = 10;

    //==========================================================================
    // Common Parameters   (Google Docs : 신뢰도 로직(claude) 표1 / (최신)consistency check(claude))
    //==========================================================================

    // 표1 체크별 U:D, N
    localparam int RU = 1, RD = 1, RN = 3;      // Range     q* = 50 %
    localparam int JU = 6, JD = 1, JN = 18;     // Jump      q* = 14 %
    localparam int NU = 1, ND = 1, NN = 3;      // Noise     q* = 50 %
    localparam int TU = 1, TD = 2, TN = 10;     // Timeout   q* = 67 %

    localparam int UPDATE_CLK_X2 = 20;          // 샘플 주기 2배 (미확정 — 실측 필요)
    localparam int DROP_N        = 2;
    localparam int HISTORY_LEN   = 10;          // noise 윈도우 = N_debounce

    // consistency : 스케일 / 계수
    localparam int S_DIST = 40,   C_DIST = 1;
    localparam int S_APSP = 1,    C_APSP = 20;
    localparam int S_ACC  = 1,    C_ACC  = 10;
    localparam int S_GYR  = 1024, C_GYR  = 3574;

    // consistency : 창
    localparam int M_WIN = 100;                 // 관계식 1
    localparam int W_WIN = 2;                   // 관계식 3~5

    // consistency : 임계치
    localparam int TH_DIST      = 360;
    localparam int TH_APSP      = 51;
    localparam int TH_ACC       = 76;
    localparam int TH_GYR       = 270;
    localparam int TH_ACC_STOP  = 4;
    localparam int TH_GYR_STOP  = 4;
    localparam int TH_ACC_TILT  = 43;
    localparam int TH_GYR_STEER = 120;

    // consistency : 카운터
    localparam int U_CONS = 1, D_CONS = 3, N_CONS = 8;

    //==========================================================================
    // Internal Signals
    //==========================================================================

    logic [NCH-1:0] range_err, jump_err, stuck_err, noise_err, timeout_err;
    logic [NCH-1:0] tm1, tm2;
    logic [NCH-1:0] stuck_mask_1s_or_20s; //뒤에 1s_or_20s 추가
    logic [NCH-1:0] stuck_mask_20s;

    logic [0:0] cons_err_distance;
    logic [0:0] cons_err_approach_speed;
    logic [2:0] cons_err_accel_x;
    logic [2:0] cons_err_accel_y;
    logic [1:0] cons_err_accel_z;
    logic [1:0] cons_err_gyro_x;
    logic [1:0] cons_err_gyro_y;
    logic [2:0] cons_err_gyro_z;

    logic [NCH-1:0] cons_err;

    //내부적으로 돌려 쓸 수 있는 consistency mask_1,2,3 이외의 신호들 추가
    logic consistency_mask_4;
    logic consistency_mask_5;
    logic consistency_mask_6;
    logic consistency_mask_7;
    logic consistency_mask_8;

    logic untrusted_accel_y;
    logic untrusted_accel_z;
    logic untrusted_distance;
    logic untrusted_approach_speed;

    assign untrusted_accel_y = range_err[CH_AY] | timeout_err[CH_AY] | stuck_err[CH_AY] | jump_err[CH_AY] | noise_err[CH_AY];
    assign untrusted_accel_z = range_err[CH_AZ] | timeout_err[CH_AZ] | stuck_err[CH_AZ] | jump_err[CH_AZ] | noise_err[CH_AZ];
    assign untrusted_distance = range_err[CH_DIST] | timeout_err[CH_DIST] | stuck_err[CH_DIST] | jump_err[CH_DIST] | noise_err[CH_DIST];
    assign untrusted_approach_speed = range_err[CH_APSP] | timeout_err[CH_APSP] | stuck_err[CH_APSP] | jump_err[CH_APSP] | noise_err[CH_APSP];
    

    assign consistency_mask_4 = (situation != 000);
    assign consistency_mask_5 = (situation != 000 || untrusted_accel_z);
    assign consistency_mask_6 = (situation != 000 || untrusted_accel_y || untrusted_accel_z);
    assign consistency_mask_7 = untrusted_approach_speed;
    assign consistency_mask_8 = untrusted_distance;






    assign cons_err[CH_DIST] = |cons_err_distance;
    assign cons_err[CH_APSP] = |cons_err_approach_speed;
    assign cons_err[CH_AX]   = |cons_err_accel_x;
    assign cons_err[CH_AY]   = |cons_err_accel_y;
    assign cons_err[CH_AZ]   = |cons_err_accel_z;
    assign cons_err[CH_GX]   = |cons_err_gyro_x;
    assign cons_err[CH_GY]   = |cons_err_gyro_y;
    assign cons_err[CH_GZ]   = |cons_err_gyro_z;
    assign cons_err[CH_TEMP] = 1'b0;
    assign cons_err[CH_HUM]  = 1'b0;
    assign cons_err[CH_LUX]  = 1'b0;

    //==========================================================================
    // Packing  --  reliability_state_t
    //==========================================================================

    reliability_state_t reliability;

    assign reliability_out = reliability;

    function automatic channel_reliability_t pack_ch(
        input logic r, input logic j, input logic s, input logic n,
        input logic c, input logic t
    );
        pack_ch.range       = r;
        pack_ch.jump        = j;
        pack_ch.stuck       = s;
        pack_ch.timeout     = t;
        pack_ch.noise       = n;
        pack_ch.consistency = c;
        pack_ch.state       = (r | t | (s+c == 2)) ? 2'b10 : (j | n | (s+c == 1)) ? 2'b01 : 2'b00;
    endfunction

    always_comb begin
        reliability.distance       = pack_ch(range_err[CH_DIST], jump_err[CH_DIST], stuck_err[CH_DIST], noise_err[CH_DIST], cons_err[CH_DIST], timeout_err[CH_DIST]);
        reliability.approach_speed = pack_ch(range_err[CH_APSP], jump_err[CH_APSP], stuck_err[CH_APSP], noise_err[CH_APSP], cons_err[CH_APSP], timeout_err[CH_APSP]);
        reliability.accel_x        = pack_ch(range_err[CH_AX],   jump_err[CH_AX],   stuck_err[CH_AX],   noise_err[CH_AX],   cons_err[CH_AX], timeout_err[CH_AX]);
        reliability.accel_y        = pack_ch(range_err[CH_AY],   jump_err[CH_AY],   stuck_err[CH_AY],   noise_err[CH_AY],   cons_err[CH_AY], timeout_err[CH_AY]);
        reliability.accel_z        = pack_ch(range_err[CH_AZ],   jump_err[CH_AZ],   stuck_err[CH_AZ],   noise_err[CH_AZ],   cons_err[CH_AZ], timeout_err[CH_AZ]);
        reliability.gyro_x         = pack_ch(range_err[CH_GX],   jump_err[CH_GX],   stuck_err[CH_GX],   noise_err[CH_GX],   cons_err[CH_GX], timeout_err[CH_GX]);
        reliability.gyro_y         = pack_ch(range_err[CH_GY],   jump_err[CH_GY],   stuck_err[CH_GY],   noise_err[CH_GY],   cons_err[CH_GY], timeout_err[CH_GY]);
        reliability.gyro_z         = pack_ch(range_err[CH_GZ],   jump_err[CH_GZ],   stuck_err[CH_GZ],   noise_err[CH_GZ],   cons_err[CH_GZ], timeout_err[CH_GZ]);
        reliability.temperature    = pack_ch(range_err[CH_TEMP], jump_err[CH_TEMP], stuck_err[CH_TEMP], noise_err[CH_TEMP], cons_err[CH_TEMP], timeout_err[CH_TEMP]);
        reliability.humidity       = pack_ch(range_err[CH_HUM],  jump_err[CH_HUM],  stuck_err[CH_HUM],  noise_err[CH_HUM],  cons_err[CH_HUM], timeout_err[CH_HUM]);
        reliability.lux            = pack_ch(range_err[CH_LUX],  jump_err[CH_LUX],  stuck_err[CH_LUX],  noise_err[CH_LUX],  cons_err[CH_LUX], timeout_err[CH_LUX]);
    end

    always_comb begin
        stuck_mask_1s_or_20s[CH_DIST]  = range_err[CH_APSP] | jump_err[CH_APSP] | stuck_err[CH_APSP] | noise_err[CH_APSP]| cons_err[CH_APSP] | timeout_err[CH_APSP] | stuck_mask_20s[CH_APSP]; //stuck_mask_20s 추가
        stuck_mask_1s_or_20s[CH_APSP]  = range_err[CH_DIST] | jump_err[CH_DIST] | stuck_err[CH_DIST] | noise_err[CH_DIST]| cons_err[CH_DIST] | timeout_err[CH_DIST] | stuck_mask_20s[CH_DIST]; //stuck_mask_20s 추가
        stuck_mask_1s_or_20s[CH_AX]    = 1'b0;
        stuck_mask_1s_or_20s[CH_AY]    = 1'b0;
        stuck_mask_1s_or_20s[CH_AZ]    = 1'b0;
        stuck_mask_1s_or_20s[CH_GX]    = 1'b0;
        stuck_mask_1s_or_20s[CH_GY]    = 1'b0;
        stuck_mask_1s_or_20s[CH_GZ]    = 1'b0;
    end

    //stuck_mask_1s->stuck_mask_1s_or_20s로 수정
    //stuck_mask_20s or 시킴



    // sample_seq, valid_s2 (신뢰도의)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out_rel       <= 1'b0;
            sample_seq_out_rel <= '0;
        end
        else begin
            valid_out_rel <= valid_s1;
            sample_seq_out_rel <= sample_seq_in;
        end
    end

    //==========================================================================
    // mask_20s -- stuck, consistency
    //==========================================================================
    mask_20s #(
        .CLK_FREQ(100000000)
    ) u_mask_20s_distance (
        .clk(clk), .rst_n(rst_n),
        .stuck_err(stuck_err[CH_DIST]),
        .consistency_error_approach_speed(),
        .stuck_mask_20s(stuck_mask_20s[CH_DIST]),
        .consistency_mask_20s_approach_speed()
    );

    mask_20s #(
        .CLK_FREQ(100000000)
    ) u_mask_20s_approach_speed (
        .clk(clk), .rst_n(rst_n),
        .stuck_err(stuck_err[CH_APSP]),
        .consistency_error_approach_speed(cons_err_approach_speed[0]),
        .stuck_mask_20s(stuck_mask_20s[CH_APSP]),
        .consistency_mask_20s_approach_speed(consistency_mask_20s_approach_speed)
    );

    //==========================================================================
    // each_sensor_check   --   sensor_data_t 채널당 1개
    //==========================================================================

    each_sensor_check #(
        .WIDTH(16), .DW(16), .TW(14), .CHANNEL_TYPE_1(1), .CHANNEL_TYPE_2(1),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(0), .RANGE_THRESHOLD_MAX(20000),
        .RANGE_USE_MIN(1'b0), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(100), .STUCK_THRESHOLD(20),
        .NOISE_THRESHOLD_1(25), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_distance (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_DIST]),
        .sensor_data        ({1'b0, sensor_data_in.distance}),
        .processed_data     (processed_data_in.delta_distance),
        .prev_processed_data(prev_processed_data_in.delta_distance),
        .trig_val_1         (processed_data_in.delta_approach_speed),
        .trig_val_2         ('0),
        .situation          (situation),
        .range_error(range_err[CH_DIST]), .jump_error(jump_err[CH_DIST]),
        .stuck_error(stuck_err[CH_DIST]), .timeout_error(timeout_err[CH_DIST]),
        .noise_error(noise_err[CH_DIST]),
        .timeout_mask_1s(tm1[CH_DIST]), .timeout_mask_2s(tm2[CH_DIST])
    );

    each_sensor_check #(
        .WIDTH(13), .DW(14), .TW(16), .CHANNEL_TYPE_1(1), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-4000), .RANGE_THRESHOLD_MAX(4000),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(10), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(3), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_approach_speed (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_APSP]),
        .sensor_data        (sensor_data_in.approach_speed),
        .processed_data     (processed_data_in.delta_approach_speed),
        .prev_processed_data(prev_processed_data_in.delta_approach_speed),
        .trig_val_1         (processed_data_in.delta_distance),
        .trig_val_2         (prev_processed_data_in.delta_distance),
        .situation          (situation),
        .range_error(range_err[CH_APSP]), .jump_error(jump_err[CH_APSP]),
        .stuck_error(stuck_err[CH_APSP]), .timeout_error(timeout_err[CH_APSP]),
        .noise_error(noise_err[CH_APSP]),
        .timeout_mask_1s(tm1[CH_APSP]), .timeout_mask_2s(tm2[CH_APSP])
    );

    each_sensor_check #(
        .WIDTH(12), .DW(13), .TW(15), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-1600), .RANGE_THRESHOLD_MAX(1600),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(2000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(500), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_accel_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_AX]),
        .sensor_data        (sensor_data_in.accel_x),
        .processed_data     (processed_data_in.delta_accel_x),
        .prev_processed_data(prev_processed_data_in.delta_accel_x),
        .trig_val_1         (processed_data_in.delta_speed_x),
        .trig_val_2         (prev_processed_data_in.delta_speed_x),
        .situation          (situation),
        .range_error(range_err[CH_AX]), .jump_error(jump_err[CH_AX]),
        .stuck_error(stuck_err[CH_AX]), .timeout_error(timeout_err[CH_AX]),
        .noise_error(noise_err[CH_AX]),
        .timeout_mask_1s(tm1[CH_AX]), .timeout_mask_2s(tm2[CH_AX])
    );

    each_sensor_check #(
        .WIDTH(12), .DW(13), .TW(15), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-1600), .RANGE_THRESHOLD_MAX(1600),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(2000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(500), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_accel_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_AY]),
        .sensor_data        (sensor_data_in.accel_y),
        .processed_data     (processed_data_in.delta_accel_y),
        .prev_processed_data(prev_processed_data_in.delta_accel_y),
        .trig_val_1         (processed_data_in.delta_speed_y),
        .trig_val_2         (prev_processed_data_in.delta_speed_y),
        .situation          (situation),
        .range_error(range_err[CH_AY]), .jump_error(jump_err[CH_AY]),
        .stuck_error(stuck_err[CH_AY]), .timeout_error(timeout_err[CH_AY]),
        .noise_error(noise_err[CH_AY]),
        .timeout_mask_1s(tm1[CH_AY]), .timeout_mask_2s(tm2[CH_AY])
    );

    each_sensor_check #(
        .WIDTH(12), .DW(13), .TW(15), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-1600), .RANGE_THRESHOLD_MAX(1600),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(2000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(500), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_accel_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_AZ]),
        .sensor_data        (sensor_data_in.accel_z),
        .processed_data     (processed_data_in.delta_accel_z),
        .prev_processed_data(prev_processed_data_in.delta_accel_z),
        .trig_val_1         (processed_data_in.delta_speed_z),
        .trig_val_2         (prev_processed_data_in.delta_speed_z),
        .situation          (situation),
        .range_error(range_err[CH_AZ]), .jump_error(jump_err[CH_AZ]),
        .stuck_error(stuck_err[CH_AZ]), .timeout_error(timeout_err[CH_AZ]),
        .noise_error(noise_err[CH_AZ]),
        .timeout_mask_1s(tm1[CH_AZ]), .timeout_mask_2s(tm2[CH_AZ])
    );

    each_sensor_check #(
        .WIDTH(16), .DW(17), .TW(17), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-16000), .RANGE_THRESHOLD_MAX(16000),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(1000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(250), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_gyro_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_GX]),
        .sensor_data        (sensor_data_in.gyro_x),
        .processed_data     (processed_data_in.delta_gyro_x),
        .prev_processed_data(prev_processed_data_in.delta_gyro_x),
        .trig_val_1         (processed_data_in.delta_incline_x),
        .trig_val_2         (prev_processed_data_in.delta_incline_x),
        .situation          (situation),
        .range_error(range_err[CH_GX]), .jump_error(jump_err[CH_GX]),
        .stuck_error(stuck_err[CH_GX]), .timeout_error(timeout_err[CH_GX]),
        .noise_error(noise_err[CH_GX]),
        .timeout_mask_1s(tm1[CH_GX]), .timeout_mask_2s(tm2[CH_GX])
    );

    each_sensor_check #(
        .WIDTH(16), .DW(17), .TW(17), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-16000), .RANGE_THRESHOLD_MAX(16000),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(1000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(250), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_gyro_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_GY]),
        .sensor_data        (sensor_data_in.gyro_y),
        .processed_data     (processed_data_in.delta_gyro_y),
        .prev_processed_data(prev_processed_data_in.delta_gyro_y),
        .trig_val_1         (processed_data_in.delta_incline_y),
        .trig_val_2         (prev_processed_data_in.delta_incline_y),
        .situation          (situation),
        .range_error(range_err[CH_GY]), .jump_error(jump_err[CH_GY]),
        .stuck_error(stuck_err[CH_GY]), .timeout_error(timeout_err[CH_GY]),
        .noise_error(noise_err[CH_GY]),
        .timeout_mask_1s(tm1[CH_GY]), .timeout_mask_2s(tm2[CH_GY])
    );

    each_sensor_check #(
        .WIDTH(16), .DW(17), .TW(17), .CHANNEL_TYPE_1(0), .CHANNEL_TYPE_2(2),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-16000), .RANGE_THRESHOLD_MAX(16000),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(1000), .STUCK_THRESHOLD(2),
        .NOISE_THRESHOLD_1(250), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(1), .STUCK_N(10),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_gyro_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_GZ]),
        .sensor_data        (sensor_data_in.gyro_z),
        .processed_data     (processed_data_in.delta_gyro_z),
        .prev_processed_data(prev_processed_data_in.delta_gyro_z),
        .trig_val_1         (processed_data_in.delta_incline_z),
        .trig_val_2         (prev_processed_data_in.delta_incline_z),
        .situation          (situation),
        .range_error(range_err[CH_GZ]), .jump_error(jump_err[CH_GZ]),
        .stuck_error(stuck_err[CH_GZ]), .timeout_error(timeout_err[CH_GZ]),
        .noise_error(noise_err[CH_GZ]),
        .timeout_mask_1s(tm1[CH_GZ]), .timeout_mask_2s(tm2[CH_GZ])
    );

    each_sensor_check #(
        .WIDTH(11), .DW(12), .TW(12), .CHANNEL_TYPE_1(2), .CHANNEL_TYPE_2(0),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(-500), .RANGE_THRESHOLD_MAX(600),
        .RANGE_USE_MIN(1'b1), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(5), .STUCK_THRESHOLD(0),
        .NOISE_THRESHOLD_1(2), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(2), .STUCK_N(15),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_temperature (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_TEMP]),
        .sensor_data        (sensor_data_in.temperature),
        .processed_data     (processed_data_in.delta_temp),
        .prev_processed_data(prev_processed_data_in.delta_temp),
        .trig_val_1         ('0),
        .trig_val_2         ('0),
        .situation          (situation),
        .range_error(range_err[CH_TEMP]), .jump_error(jump_err[CH_TEMP]),
        .stuck_error(stuck_err[CH_TEMP]), .timeout_error(timeout_err[CH_TEMP]),
        .noise_error(noise_err[CH_TEMP]),
        .timeout_mask_1s(tm1[CH_TEMP]), .timeout_mask_2s(tm2[CH_TEMP])
    );

    each_sensor_check #(
        .WIDTH(8), .DW(8), .TW(8), .CHANNEL_TYPE_1(2), .CHANNEL_TYPE_2(0),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(0), .RANGE_THRESHOLD_MAX(100),
        .RANGE_USE_MIN(1'b0), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(5), .STUCK_THRESHOLD(0),
        .NOISE_THRESHOLD_1(2), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(2), .STUCK_N(15),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_humidity (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_HUM]),
        .sensor_data        ({1'b0, sensor_data_in.humidity}),
        .processed_data     (processed_data_in.delta_hum),
        .prev_processed_data(prev_processed_data_in.delta_hum),
        .trig_val_1         ('0),
        .trig_val_2         ('0),
        .situation          (situation),
        .range_error(range_err[CH_HUM]), .jump_error(jump_err[CH_HUM]),
        .stuck_error(stuck_err[CH_HUM]), .timeout_error(timeout_err[CH_HUM]),
        .noise_error(noise_err[CH_HUM]),
        .timeout_mask_1s(tm1[CH_HUM]), .timeout_mask_2s(tm2[CH_HUM])
    );

    each_sensor_check #(
        .WIDTH(19), .DW(19), .TW(19), .CHANNEL_TYPE_1(2), .CHANNEL_TYPE_2(0),
        .UPDATE_CLK_X2(UPDATE_CLK_X2), .DROP_N(DROP_N), .HISTORY(HISTORY_LEN),
        .RANGE_THRESHOLD_MIN(0), .RANGE_THRESHOLD_MAX(130000),
        .RANGE_USE_MIN(1'b0), .RANGE_USE_MAX(1'b1),
        .JUMP_THRESHOLD(20000), .STUCK_THRESHOLD(0),
        .NOISE_THRESHOLD_1(5000), .NOISE_THRESHOLD_2(7),
        .RANGE_U(RU),   .RANGE_D(RD),   .RANGE_N(RN),
        .JUMP_U(JU),    .JUMP_D(JD),    .JUMP_N(JN),
        .STUCK_U(1), .STUCK_D(2), .STUCK_N(15),
        .TIMEOUT_U(TU), .TIMEOUT_D(TD), .TIMEOUT_N(TN)
    ) u_chk_lux (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1), .clk_cnt(clk_cnt),
        .stuck_mask         (stuck_mask_1s_or_20s[CH_LUX]),
        .sensor_data        ({1'b0, sensor_data_in.lux}),
        .processed_data     (processed_data_in.delta_lux),
        .prev_processed_data(prev_processed_data_in.delta_lux),
        .trig_val_1         ('0),
        .trig_val_2         ('0),
        .situation          (situation),
        .range_error(range_err[CH_LUX]), .jump_error(jump_err[CH_LUX]),
        .stuck_error(stuck_err[CH_LUX]), .timeout_error(timeout_err[CH_LUX]),
        .noise_error(noise_err[CH_LUX]),
        .timeout_mask_1s(tm1[CH_LUX]), .timeout_mask_2s(tm2[CH_LUX])
    );

    //==========================================================================
    // consistency_check   --   채널당 관계식 개수만큼
    //   관계식 번호는 (최신)consistency check(claude) 기준
    //==========================================================================

    // 관계식 1
    consistency_check #(
        .WIDTH(16), .S(S_DIST), .CONSISTENCY_THRESHOLD(TH_DIST),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_distance (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_DIST]),
        .consistency_mask (consistency_mask_7),
        .sensor_data      ({1'b0, sensor_data_in.distance}),
        .pred_data        (pred_data_in.pred_distance),
        .consistency_error(cons_err_distance[0])
    );

    // 관계식 2
    consistency_check #(
        .WIDTH(13), .S(S_APSP), .CONSISTENCY_THRESHOLD(TH_APSP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_approach_speed (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_APSP]),
        .consistency_mask (consistency_mask_8),
        .sensor_data      (sensor_data_in.approach_speed),
        .pred_data        (pred_data_in.pred_approach_speed),
        .consistency_error(cons_err_approach_speed[0])
    );

    // 관계식 3
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_accel_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AX]),
        .consistency_mask (consistency_mask_1),
        .sensor_data      (sensor_data_in.accel_x),
        .pred_data        (pred_data_in.pred_accel_x_1),
        .consistency_error(cons_err_accel_x[0])
    );

    // 관계식 9
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_accel_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AX]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.accel_x),
        .pred_data        (pred_data_in.pred_accel_x_2),
        .consistency_error(cons_err_accel_x[1])
    );

    // 관계식 16
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC_TILT),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_3_accel_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AX]),
        .consistency_mask (consistency_mask_6),
        .sensor_data      (sensor_data_in.accel_x),
        .pred_data        (pred_data_in.pred_accel_x_3),
        .consistency_error(cons_err_accel_x[2])
    );

    // 관계식 4
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_accel_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AY]),
        .consistency_mask (consistency_mask_1),
        .sensor_data      (sensor_data_in.accel_y),
        .pred_data        (pred_data_in.pred_accel_y_1),
        .consistency_error(cons_err_accel_y[0])
    );

    // 관계식 10
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_accel_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AY]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.accel_y),
        .pred_data        (pred_data_in.pred_accel_y_2),
        .consistency_error(cons_err_accel_y[1])
    );

    // 관계식 15
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC_TILT),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_3_accel_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AY]),
        .consistency_mask (consistency_mask_5),
        .sensor_data      (sensor_data_in.accel_y),
        .pred_data        (pred_data_in.pred_accel_y_3),
        .consistency_error(cons_err_accel_y[2])
    );

    // 관계식 5
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_accel_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AZ]),
        .consistency_mask (consistency_mask_1),
        .sensor_data      (sensor_data_in.accel_z),
        .pred_data        (pred_data_in.pred_accel_z_1),
        .consistency_error(cons_err_accel_z[0])
    );

    // 관계식 11
    consistency_check #(
        .WIDTH(12), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_ACC_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_accel_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_AZ]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.accel_z),
        .pred_data        (pred_data_in.pred_accel_z_2),
        .consistency_error(cons_err_accel_z[1])
    );

    // 관계식 6
    consistency_check #(
        .WIDTH(16), .S(S_GYR), .CONSISTENCY_THRESHOLD(TH_GYR),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_gyro_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GX]),
        .consistency_mask (consistency_mask_2),
        .sensor_data      (sensor_data_in.gyro_x),
        .pred_data        (pred_data_in.pred_gyro_x_1),
        .consistency_error(cons_err_gyro_x[0])
    );

    // 관계식 12
    consistency_check #(
        .WIDTH(16), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_GYR_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_gyro_x (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GX]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.gyro_x),
        .pred_data        (pred_data_in.pred_gyro_x_2),
        .consistency_error(cons_err_gyro_x[1])
    );

    // 관계식 7
    consistency_check #(
        .WIDTH(16), .S(S_GYR), .CONSISTENCY_THRESHOLD(TH_GYR),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_gyro_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GY]),
        .consistency_mask (consistency_mask_2),
        .sensor_data      (sensor_data_in.gyro_y),
        .pred_data        (pred_data_in.pred_gyro_y_1),
        .consistency_error(cons_err_gyro_y[0])
    );

    // 관계식 13
    consistency_check #(
        .WIDTH(16), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_GYR_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_gyro_y (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GY]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.gyro_y),
        .pred_data        (pred_data_in.pred_gyro_y_2),
        .consistency_error(cons_err_gyro_y[1])
    );

    // 관계식 8
    consistency_check #(
        .WIDTH(16), .S(S_GYR), .CONSISTENCY_THRESHOLD(TH_GYR),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_1_gyro_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GZ]),
        .consistency_mask (consistency_mask_2),
        .sensor_data      (sensor_data_in.gyro_z),
        .pred_data        (pred_data_in.pred_gyro_z_1),
        .consistency_error(cons_err_gyro_z[0])
    );

    // 관계식 14
    consistency_check #(
        .WIDTH(16), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_GYR_STOP),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_2_gyro_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GZ]),
        .consistency_mask (consistency_mask_4),
        .sensor_data      (sensor_data_in.gyro_z),
        .pred_data        (pred_data_in.pred_gyro_z_2),
        .consistency_error(cons_err_gyro_z[1])
    );

    // 관계식 17
    consistency_check #(
        .WIDTH(16), .S(S_ACC), .CONSISTENCY_THRESHOLD(TH_GYR_STEER),
        .CONSISTENCY_U(U_CONS), .CONSISTENCY_D(D_CONS), .CONSISTENCY_N(N_CONS)
    ) u_cons_3_gyro_z (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .timeout_mask_2s  (tm2[CH_GZ]),
        .consistency_mask (consistency_mask_3),
        .sensor_data      (sensor_data_in.gyro_z),
        .pred_data        (pred_data_in.pred_gyro_z_3),
        .consistency_error(cons_err_gyro_z[2])
    );    
    
    

endmodule
