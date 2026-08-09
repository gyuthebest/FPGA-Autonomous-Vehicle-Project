/*
문서대로라면
1 
.FUNCTION_NUM(1), .S(40), .C(1), .M(100), .TH(360), .MASK_TH1(20000)

2
.FUNCTION_NUM(2), .S(1), .C(20), .TH(51)

3, 4, 5
.FUNCTION_NUM(3), .S(1), .C(10), .W(2), .TH(76), .MASK_TH1(2990)

6, 7, 8
.FUNCTION_NUM(6), .S(1024), .C(3574), .TH(270), .MASK_TH1(300), .MASK_TH2(2000)

9, 10, 11
.FUNCTION_NUM(9), .TH(4), .MASK_TH1(10)

12, 13, 14
.FUNCTION_NUM(12), .TH(4), .MASK_TH1(10)

15, 16
.FUNCTION_NUM(15), .TH(43), .MASK_TH1(10)

17
.FUNCTION_NUM(17), .TH(120), .MASK_TH1(2990), .MASK_TH2(100)
*/

timescale 1ns / 1ps

import types_pkg::*;

module consistency_check #(
    parameter int FUNCTION_NUM = 1,
    parameter int UPDATE_FREQ = 20,
    parameter int CLK_FREQ = 100000000,

    parameter int S = 1,
    parameter int C = 1,
    parameter int M = 100,
    parameter int W = 2,
    parameter int TH = 100,
    
    
    parameter int MASK_TH1 = 0,
    parameter int MASK_TH2 = 0,

    parameter int unsigned U = 1,
    parameter int unsigned D = 3,
    parameter int unsigned N = 8
)(
    input logic clk,
    input logic rst_n,
    input logic valid_s1,
    input logic sample_seq,
    
    input logic warmup,
    input logic is_timeout,
    input reliability_state_t reliability_in,
    input sensor_data_t sensor_data_in,
    input sim_data_t sim_data_in,
    input sensor_data_t prev_sensor_data_in,
    input reliability_state_t rel_in,
    
    output logic consistency
    
);
    
    localparam int G = 981;
    localparam int L = 272;
    localparam int LUT_SH = 10;
    localparam int INCL_MAX = 3000;
    localparam int STEER_MAX = 100;
    localparam int DELTA_MAX = 35;


    logic [10:0] LUT_SIN [0:24];
    logic [10:0] LUT_COS [0:24];
    logic [10:0] LUT_TAN [0:24];
    logic [11:0] LUT_STEER [0:13];

    always_comb begin
        LUT_SIN[0] = 11'd0;
        LUT_SIN[1] = 11'd23;
        LUT_SIN[2] = 11'd46;
        LUT_SIN[3] = 11'd69;
        LUT_SIN[4] = 11'd91;
        LUT_SIN[5] = 11'd114;
        LUT_SIN[6] = 11'd137;
        LUT_SIN[7] = 11'd159;
        LUT_SIN[8] = 11'd182;
        LUT_SIN[9] = 11'd205;
        LUT_SIN[10] = 11'd227;
        LUT_SIN[11] = 11'd249;
        LUT_SIN[12] = 11'd271;
        LUT_SIN[13] = 11'd293;
        LUT_SIN[14] = 11'd315;
        LUT_SIN[15] = 11'd337;
        LUT_SIN[16] = 11'd358;
        LUT_SIN[17] = 11'd380;
        LUT_SIN[18] = 11'd401;
        LUT_SIN[19] = 11'd422;
        LUT_SIN[20] = 11'd442;
        LUT_SIN[21] = 11'd463;
        LUT_SIN[22] = 11'd483;
        LUT_SIN[23] = 11'd503;
        LUT_SIN[24] = 11'd523;
   
        LUT_COS[0] = 11'd1024;
        LUT_COS[1] = 11'd1024;
        LUT_COS[2] = 11'd1023;
        LUT_COS[3] = 11'd1022;
        LUT_COS[4] = 11'd1020;
        LUT_COS[5] = 11'd1018;
        LUT_COS[6] = 11'd1015;
        LUT_COS[7] = 11'd1012;
        LUT_COS[8] = 11'd1008;
        LUT_COS[9] = 11'd1003;
        LUT_COS[10] = 11'd999;
        LUT_COS[11] = 11'd993;
        LUT_COS[12] = 11'd987;
        LUT_COS[13] = 11'd981;
        LUT_COS[14] = 11'd974;
        LUT_COS[15] = 11'd967;
        LUT_COS[16] = 11'd959;
        LUT_COS[17] = 11'd951;
        LUT_COS[18] = 11'd942;
        LUT_COS[19] = 11'd933;
        LUT_COS[20] = 11'd923;
        LUT_COS[21] = 11'd913;
        LUT_COS[22] = 11'd903;
        LUT_COS[23] = 11'd892;
        LUT_COS[24] = 11'd880;
    
        LUT_TAN[0] = 11'd0;
        LUT_TAN[1] = 11'd23;
        LUT_TAN[2] = 11'd46;
        LUT_TAN[3] = 11'd69;
        LUT_TAN[4] = 11'd92;
        LUT_TAN[5] = 11'd115;
        LUT_TAN[6] = 11'd138;
        LUT_TAN[7] = 11'd161;
        LUT_TAN[8] = 11'd185;
        LUT_TAN[9] = 11'd209;
        LUT_TAN[10] = 11'd233;
        LUT_TAN[11] = 11'd257;
        LUT_TAN[12] = 11'd281;
        LUT_TAN[13] = 11'd306;
        LUT_TAN[14] = 11'd331;
        LUT_TAN[15] = 11'd357;
        LUT_TAN[16] = 11'd382;
        LUT_TAN[17] = 11'd409;
        LUT_TAN[18] = 11'd436;
        LUT_TAN[19] = 11'd463;
        LUT_TAN[20] = 11'd491;
        LUT_TAN[21] = 11'd519;
        LUT_TAN[22] = 11'd548;
        LUT_TAN[23] = 11'd578;
        LUT_TAN[24] = 11'd608;

        LUT_STEER[0] = 12'd0;
        LUT_STEER[1] = 12'd184;
        LUT_STEER[2] = 12'd369;
        LUT_STEER[3] = 12'd556;
        LUT_STEER[4] = 12'd745;
        LUT_STEER[5] = 12'd939;
        LUT_STEER[6] = 12'd1137;
        LUT_STEER[7] = 12'd1341;
        LUT_STEER[8] = 12'd1552;
        LUT_STEER[9] = 12'd1772;
        LUT_STEER[10] = 12'd2002;
        LUT_STEER[11] = 12'd2244;
        LUT_STEER[12] = 12'd2501;
        LUT_STEER[13] = 12'd2776;
    end


    //Fixed-point Linear Interpolator이라네요.
    function automatic logic signed [11:0] get_sin(input logic signed [15:0] angle);
        logic [15:0] abs_a;
        logic [4:0] idx;
        logic [7:0] rem;
        logic signed [11:0] val0, val1, val;
        abs_a = (angle < 0) ? -angle : angle;
        if (abs_a > INCL_MAX) abs_a = INCL_MAX;
        idx = abs_a >> 7;
        rem = abs_a & 127;
        if (idx >= 24) val = LUT_SIN[24];
        else begin
            val0 = LUT_SIN[idx];
            val1 = LUT_SIN[idx+1];
            val = val0 + (((val1 - val0) * rem) >>> 7);
        end
        return (angle < 0) ? -val : val;
    endfunction

    function automatic logic signed [11:0] get_cos(input logic signed [15:0] angle);
        logic [15:0] abs_a;
        logic [4:0] idx;
        logic [7:0] rem;
        logic signed [11:0] val0, val1, val;
        abs_a = (angle < 0) ? -angle : angle;
        if (abs_a > INCL_MAX) abs_a = INCL_MAX;
        idx = abs_a >> 7;
        rem = abs_a & 127;
        if (idx >= 24) val = LUT_COS[24];
        else begin
            val0 = LUT_COS[idx];
            val1 = LUT_COS[idx+1];
            val = val0 - (((val0 - val1) * rem) >>> 7);
        end
        return val;
    endfunction

    function automatic logic signed [11:0] get_tan(input logic signed [15:0] angle);
        logic [15:0] abs_a;
        logic [4:0] idx;
        logic [7:0] rem;
        logic signed [11:0] val0, val1, val;
        abs_a = (angle < 0) ? -angle : angle;
        if (abs_a > INCL_MAX) abs_a = INCL_MAX;
        idx = abs_a >> 7;
        rem = abs_a & 127;
        if (idx >= 24) val = LUT_TAN[24];
        else begin
            val0 = LUT_TAN[idx];
            val1 = LUT_TAN[idx+1];
            val = val0 + (((val1 - val0) * rem) >>> 7);
        end
        return (angle < 0) ? -val : val;
    endfunction

    function automatic logic signed [11:0] get_steer_lut(input logic signed [7:0] steering);
        logic [7:0] abs_a;
        logic [3:0] idx;
        logic [3:0] rem;
        logic signed [11:0] val0, val1, val;
        abs_a = (steering < 0) ? -steering : steering;
        if (abs_a > STEER_MAX) abs_a = STEER_MAX;
        idx = abs_a >> 3;
        rem = abs_a & 7;
        if (idx >= 13) val = LUT_STEER[13];
        else begin
            val0 = LUT_STEER[idx];
            val1 = LUT_STEER[idx+1];
            val = val0 + (((val1 - val0) * rem) >>> 3);
        end
        return (steering < 0) ? -val : val;
    endfunction











   

    // W>=2 과거값 저장 (식 3, 4, 5에 쓰임)
    logic signed [13:0] speed_x_preeev [2:W];
    logic signed [13:0] speed_y_preeev [2:W];
    logic signed [13:0] speed_z_preeev [2:W];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 2; i <= W; i++) begin
                speed_x_preeev[i] <= '0;
                speed_y_preeev[i] <= '0;
                speed_z_preeev[i] <= '0;
            end
        end 
        else if (valid_s1) begin
            speed_x_preeev[2] <= prev_sensor_data_in.speed_x;
            speed_y_preeev[2] <= prev_sensor_data_in.speed_y;
            speed_z_preeev[2] <= prev_sensor_data_in.speed_z;
            
            for (int i = 3; i <= W; i++) begin
                speed_x_preeev[i] <= speed_x_preeev[i-1];
                speed_y_preeev[i] <= speed_y_preeev[i-1];
                speed_z_preeev[i] <= speed_z_preeev[i-1];
            end
        end
    end


    // Absolute value
    function automatic logic [31:0] abs(input logic signed [31:0] val);
        return (val < 0) ? -val : val;
    endfunction


    // Mask
    logic mask, prev_mask, mask_cancel;
    logic untrusted_distance, untrusted_appspeed, standstill;
    
    assign untrusted_distance = rel_in.distance.range | rel_in.distance.jump | rel_in.distance.stuck | rel_in.distance.noise;
    assign untrusted_appspeed = rel_in.approach_speed.range | rel_in.approach_speed.jump | rel_in.approach_speed.stuck | rel_in.approach_speed.noise;
    
    assign standstill = (abs(sensor_data_in.speed_x) <= MASK_TH1) && (abs(sensor_data_in.speed_y) <= MASK_TH1) && (abs(sensor_data_in.speed_z) <= MASK_TH1);

    always_comb begin
        mask = is_timeout | warmup;
        case (FUNCTION_NUM)
            1: mask = mask | (sensor_data_in.distance == MASK_TH1) | untrusted_appspeed;
            2: mask = mask | untrusted_distance;
            3,4,5: mask = mask | (abs(sensor_data_in.incline_x) >= MASK_TH1) | (abs(sensor_data_in.incline_y) >= MASK_TH1);
            6,7,8: mask = mask | (abs(sensor_data_in.incline_x) > MASK_TH1) | (abs(sensor_data_in.incline_y) > MASK_TH1) | (abs(sensor_data_in.gyro_z) > MASK_TH2);
            9,10,11,12,13,14: mask = mask | !standstill;
            15,16: mask = mask | !standstill;
            17: mask = mask | (sensor_data_in.speed_x < MASK_TH2) | (abs(sensor_data_in.incline_x) >= MASK_TH1) | (abs(sensor_data_in.incline_y) >= MASK_TH1);
        endcase
    end

    // Mask 해제시 재시드를 위함
    always_ff @(posedge clk) begin
        if (!rst_n) 
            prev_mask <= 1'b1;
        else if (valid_s1) 
            prev_mask <= mask;
    end
    
    assign mask_cancel = (prev_mask == 1'b1 && mask == 1'b0);


    // Gravity (오일러 각 회전 변환)
    logic signed [11:0] sin_x, cos_x, sin_y, cos_y, tan_x, tan_y, steer_lut_val;
    logic signed [31:0] grav_x, grav_y, grav_z;

    always_comb begin
        sin_x = get_sin(sensor_data_in.incline_x);
        cos_x = get_cos(sensor_data_in.incline_x);
        tan_x = get_tan(sensor_data_in.incline_x);
        
        sin_y = get_sin(sensor_data_in.incline_y);
        cos_y = get_cos(sensor_data_in.incline_y);
        tan_y = get_tan(sensor_data_in.incline_y);
        
        steer_lut_val = get_steer_lut(sim_data_in.steering);

        grav_x = -(G * sin_y) >>> LUT_SH;
        grav_y = (G * sin_x * cos_y) >>> (2 * LUT_SH);
        grav_z = (G * cos_x * cos_y) >>> (2 * LUT_SH);
    end


    // Prediction, Residual (consistency check의 예측값, 예측값과 비교값의 차이)
    logic signed [31:0] pred, actual, residual;
    logic [31:0] abs_residual;
    logic signed [31:0] d_pred;

    always_comb begin
        pred = '0;
        actual = '0;
        case (FUNCTION_NUM)
            1: begin
                pred = d_pred;
                actual = sensor_data_in.distance * S;
            end
            2: begin
                pred = -(({1'b0, sensor_data_in.distance}) - ({1'b0, prev_sensor_data_in.distance})) * C;
                actual = sensor_data_in.approach_speed * S;
            end
            3: begin
                pred = (sensor_data_in.speed_x - speed_x_preeev[W]) * C + grav_x * S;
                actual = sensor_data_in.accel_x * S;
            end
            4: begin
                pred = (sensor_data_in.speed_y - speed_y_preeev[W]) * C + grav_y * S;
                actual = sensor_data_in.accel_y * S;
            end
            5: begin
                pred = (sensor_data_in.speed_z - speed_z_preeev[W]) * C + grav_z * S;
                actual = sensor_data_in.accel_z * S;
            end
            6: begin
                pred = (sensor_data_in.incline_x - prev_sensor_data_in.incline_x) * C;
                actual = sensor_data_in.gyro_x * S;
            end
            7: begin
                pred = (sensor_data_in.incline_y - prev_sensor_data_in.incline_y) * C;
                actual = sensor_data_in.gyro_y * S;
            end
            8: begin
                pred = (sensor_data_in.incline_z - prev_sensor_data_in.incline_z) * C;
                actual = sensor_data_in.gyro_z * S;
            end
            9: begin
                pred = grav_x;
                actual = sensor_data_in.accel_x;
            end
            10: begin
                pred = grav_y;
                actual = sensor_data_in.accel_y;
            end
            11: begin
                pred = grav_z;
                actual = sensor_data_in.accel_z;
            end
            12: begin
                pred = 0;
                actual = sensor_data_in.gyro_x;
            end
            13: begin
                pred = 0;
                actual = sensor_data_in.gyro_y;
            end
            14: begin
                pred = 0;
                actual = sensor_data_in.gyro_z;
            end
            15: begin
                pred = (sensor_data_in.accel_z * tan_x) >>> LUT_SH;
                actual = sensor_data_in.accel_y;
            end
            16: begin // Alpha-Max Beta-Min approximation이라네요.
                logic signed [31:0] abs_y, abs_z;
                logic signed [31:0] abs_max, abs_min;
                logic signed [31:0] approx1, approx2, approx_sqrt;
                
                abs_y = abs(sensor_data_in.accel_y);
                abs_z = abs(sensor_data_in.accel_z);
                
                // 1. Max, Min 구분
                abs_max = (abs_y > abs_z) ? abs_y : abs_z;
                abs_min = (abs_y > abs_z) ? abs_z : abs_y;
                
                // 2. 시프트 연산으로 두 가지 근사치 계산
                approx1 = abs_max + (abs_min >> 3);
                approx2 = abs_max - (abs_max >> 3) + (abs_min >> 1);
                
                // 3. 둘 중 더 큰 값을 선택
                approx_sqrt = (approx1 > approx2) ? approx1 : approx2;
                
                pred = (-approx_sqrt * tan_y) >>> LUT_SH;
                actual = sensor_data_in.accel_x;
            end
            17: begin
                pred = (sensor_data_in.speed_x * steer_lut_val) >>> LUT_SH;
                actual = sensor_data_in.gyro_z;
            end
        endcase
        residual = actual - pred;
        abs_residual = abs(residual);
    end


    // ff
    logic [7:0] win;
    logic [$clog2(N+1)-1:0] consistency_cnt;

    assign consistency = (consistency_cnt >= N);
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            win <= '0;
            consistency_cnt <= '0;
            d_pred <= '0;
        end 
        else if (valid_s1) begin
            if (is_timeout || warmup || (mask_cancel && FUNCTION_NUM == 1)) begin
                win <= '0;
                d_pred <= sensor_data_in.distance * S;
            end 
            else if (!mask) begin
                if (abs_residual > TH)
                    consistency_cnt <= consistency_cnt + U; // 오버플로우 날까? each_sensor_checker.sv대로 일단 해놓긴 하였음
                else
                    consistency_cnt <= (consistency_cnt < D) ? '0 : consistency_cnt - D;
                
                if (win == M - 1) begin
                    win <= '0;
                    d_pred <= sensor_data_in.distance * S;
                end 
                else begin
                    win <= win + 1;
                    if (FUNCTION_NUM == 1)
                        d_pred <= d_pred - (sensor_data_in.approach_speed + prev_sensor_data_in.approach_speed) * C;
                end
            end
        end
    end

endmodule
