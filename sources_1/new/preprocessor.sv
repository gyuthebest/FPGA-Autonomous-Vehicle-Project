`timescale 1ns / 1ps

import types_pkg::*;

module preprocessor (
    input  logic clk,//
    input  logic rst_n,//

    //------------------------------------------------------------
    // Inputs
    //------------------------------------------------------------

    input  sensor_data_t   sensor_data_in, //
    input  sim_data_t      sim_data_in, //
    input  logic [31:0]    sample_seq_in, //
    input  logic           timeout_mask_1s, //
    input  logic           consistency_mask_1s_approach_speed, //
    input  logic           consistency_mask_20s_approach_speed, //

    //------------------------------------------------------------
    // Outputs
    //------------------------------------------------------------

    output sensor_data_t    sensor_data_out, //
    output sim_data_t       sim_data_out, //
    output processed_data_t processed_data_out, //
    output processed_data_t prev_processed_data_out, //
    output pred_data_t      pred_data_out, //
    output logic            valid_s1, //
    output logic [31:0]     sample_seq_out, //
    output logic [31:0]     clk_cnt, //
    output logic            consistency_mask_1, //
    output logic            consistency_mask_2, //
    output logic            consistency_mask_3 //
);
    localparam HISTORY = 10;
    localparam int S_DIST  = 40;
    localparam int C_DIST  = 1;
    localparam int S_APSP   = 1;
    localparam int C_APSP   = 20;
    localparam int S_ACC    = 1;
    localparam int C_ACC    = 10;
    localparam int S_GYR    = 1024;
    localparam int C_GYR    = 3574;
    localparam M = 100;
    localparam W = 2;
    localparam int G = 981;
    localparam int L = 272;
    localparam int LUT_SH = 10;
    localparam int INCL_MAX = 3000;
    localparam int STEER_MAX = 100;
    localparam int CLAMP_TH  = 2990;
    localparam int LVL_TH    = 300;
    localparam int YAW_TH    = 2000;
    localparam int STOP_TH   = 10;
    localparam int SPD_MIN   = 100;


    //------------------------------------------------------------
    // Calculate valid_s0
    //------------------------------------------------------------
    
    logic valid_s0;

    always_comb begin
        if (sample_seq_in != sample_seq_out) begin
            valid_s0 = 1'b1;
        end
        else begin
            valid_s0 = 1'b0;
        end
    end

    sensor_data_t    prev_sensor_data_out;
    processed_data_t processed_data;
    processed_data_t prev_processed_data;

    always_comb begin

        processed_data = processed_data_out;
        prev_processed_data = prev_processed_data_out;

        // Ignore two sample (Can't calculate delta)
        if(valid_s0) begin

            //--------------------------------------------------------
            // Distance Delta
            //--------------------------------------------------------
            
            processed_data.delta_distance =
                $signed({1'b0, sensor_data_in.distance}) -
                $signed({1'b0, sensor_data_out.distance});

            //--------------------------------------------------------
            // Approach Speed Delta
            //--------------------------------------------------------

            processed_data.delta_approach_speed =
                sensor_data_in.approach_speed -
                sensor_data_out.approach_speed;


            //--------------------------------------------------------
            // Acceleration Delta
            //--------------------------------------------------------

            processed_data.delta_accel_x =
                sensor_data_in.accel_x -
                sensor_data_out.accel_x;

            processed_data.delta_accel_y =
                sensor_data_in.accel_y -
                sensor_data_out.accel_y;

            processed_data.delta_accel_z =
                sensor_data_in.accel_z -
                sensor_data_out.accel_z;

            //--------------------------------------------------------
            // Gyroscope Delta
            //--------------------------------------------------------

            processed_data.delta_gyro_x =
                sensor_data_in.gyro_x -
                sensor_data_out.gyro_x;

            processed_data.delta_gyro_y =
                sensor_data_in.gyro_y -
                sensor_data_out.gyro_y;

            processed_data.delta_gyro_z =
                sensor_data_in.gyro_z -
                sensor_data_out.gyro_z;

            //--------------------------------------------------------
            // Temperature Delta
            //--------------------------------------------------------

            processed_data.delta_temp =
                sensor_data_in.temperature -
                sensor_data_out.temperature;

            //--------------------------------------------------------
            // Humidity Delta
            //--------------------------------------------------------

            processed_data.delta_hum =
                $signed({1'b0, sensor_data_in.humidity}) -
                $signed({1'b0, sensor_data_out.humidity});

            //--------------------------------------------------------
            // Lux Delta
            //--------------------------------------------------------

            processed_data.delta_lux =
                $signed({1'b0, sensor_data_in.lux}) -
                $signed({1'b0, sensor_data_out.lux});

            //--------------------------------------------------------
            // Speed Delta
            //--------------------------------------------------------

            processed_data.delta_speed_x =
                sim_data_in.speed_x -
                sim_data_out.speed_x;

            processed_data.delta_speed_y =
                sim_data_in.speed_y -
                sim_data_out.speed_y;

            processed_data.delta_speed_z =
                sim_data_in.speed_z -
                sim_data_out.speed_z;

            //--------------------------------------------------------
            // Incline Delta
            //--------------------------------------------------------

            processed_data.delta_incline_x =
                sim_data_in.incline_x -
                sim_data_out.incline_x;

            processed_data.delta_incline_y =
                sim_data_in.incline_y -
                sim_data_out.incline_y;

            processed_data.delta_incline_z =
                sim_data_in.incline_z -
                sim_data_out.incline_z;

            //--------------------------------------------------------
            // Steering Delta
            //--------------------------------------------------------
        
            processed_data.delta_steering =
                sim_data_in.steering -
                sim_data_out.steering;

            //--------------------------------------------------------
            // Previous Distance Delta
            //--------------------------------------------------------
            
            prev_processed_data.delta_distance =
                $signed({1'b0, sensor_data_out.distance}) -
                $signed({1'b0, prev_sensor_data_out.distance});
                
            //--------------------------------------------------------
            // Previous Approach Speed Delta
            //--------------------------------------------------------

            prev_processed_data.delta_approach_speed =
                sensor_data_out.approach_speed -
                prev_sensor_data_out.approach_speed;

            //--------------------------------------------------------
            // Previous Acceleration Delta
            //--------------------------------------------------------

            prev_processed_data.delta_accel_x =
                sensor_data_out.accel_x -
                prev_sensor_data_out.accel_x;

            prev_processed_data.delta_accel_y =
                sensor_data_out.accel_y -
                prev_sensor_data_out.accel_y;

            prev_processed_data.delta_accel_z =
                sensor_data_out.accel_z -
                prev_sensor_data_out.accel_z;

            //--------------------------------------------------------
            // Previous Gyroscope Delta
            //--------------------------------------------------------

            prev_processed_data.delta_gyro_x =
                sensor_data_out.gyro_x -
                prev_sensor_data_out.gyro_x;

            prev_processed_data.delta_gyro_y =
                sensor_data_out.gyro_y -
                prev_sensor_data_out.gyro_y;

            prev_processed_data.delta_gyro_z =
                sensor_data_out.gyro_z -
                prev_sensor_data_out.gyro_z;

            //--------------------------------------------------------
            // Previous Temperature Delta
            //--------------------------------------------------------

            prev_processed_data.delta_temp =
                sensor_data_out.temperature -
                prev_sensor_data_out.temperature;

            //--------------------------------------------------------
            // Previous Humidity Delta
            //--------------------------------------------------------

            prev_processed_data.delta_hum =
                $signed({1'b0, sensor_data_out.humidity}) -
                $signed({1'b0, prev_sensor_data_out.humidity});

            //--------------------------------------------------------
            // Previous Lux Delta
            //--------------------------------------------------------

            prev_processed_data.delta_lux =
                $signed({1'b0, sensor_data_out.lux}) -
                $signed({1'b0, prev_sensor_data_out.lux});
        end
    end
        
    logic [10:0] LUT_SIN [25];
    logic [10:0] LUT_COS [25];
    logic [10:0] LUT_TAN [25];
    logic [11:0] LUT_STEER [14];

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

    // LUT_STEER 의 상위 항목(2244 / 2501 / 2776)은 signed 12비트 상한 2047 을
    // 넘는다.  중간 변수를 signed [11:0] 으로 두면 |steering| >= 80 (idx >= 10,
    // val0 또는 val1 이 2047 초과) 구간에서 값이 음수로 뒤집혀 조향 기준값의
    // 부호가 반대가 된다.  실측(pl_capture_20260814_183046): steering 91 에서
    // RTL 이 -1756, 모델이 +2340 -> 관계식 17 이 잘못 확정되어 gyro_z 가
    // DEGRADED 로 떴다.  LUT_STEER 최대 2776 이므로 signed 13비트로 넓힌다.
    function automatic logic signed [12:0] get_steer_lut(input logic signed [7:0] steering);
        logic [7:0] abs_a;
        logic [3:0] idx;
        logic [3:0] rem;
        logic signed [12:0] val0, val1, val;
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

    // Absolute value
    function automatic logic [31:0] abs(input logic signed [31:0] val);
        return (val < 0) ? -val : val;
    endfunction

    logic signed [11:0] sin_x, cos_x, sin_y, cos_y, tan_x, tan_y;
    // steer LUT 은 2776 까지 올라가므로 signed 12비트로는 부족하다 (get_steer_lut 주석).
    logic signed [12:0] steer_lut_val;
    logic signed [31:0] grav_x, grav_y, grav_z;
    // Verilog sizes a product as max(operand, context) width, not the sum, so
    // a bare (a * b) >>> LUT_SH inside a narrow assignment wraps the product
    // *before* the shift.  pred_gyro_z_3 (16-bit LHS, 14x12 product) lost the
    // steering prediction entirely this way -- board capture matched a model
    // that truncated the product to 16 bits on 38973/38973 comparisons.
    // pred_accel_y_3 (12-bit LHS, 12x12 product) has the same defect; it is
    // masked while driving by consistency_mask_5 and shows up only on a slope
    // at standstill.  Compute both products at full width first.
    logic signed [31:0] steer_yaw_pred;      // 관계식 17 : speed_x * steer LUT
    logic signed [31:0] accel_y_tilt_pred;   // 관계식 15 : accel_z * tan(incline_x)
    // 관계식 4 (횡가속도) 는 a_y = dv_y/dt + g_y 만 기준값으로 삼고 있었다.
    // 차체 좌표계에서 실제 횡가속도에는 구심항 v_x*w_z 가 반드시 들어간다.
    // 정상 선회에서는 body-frame 횡속도가 거의 변하지 않아 dv_y/dt ~ 0 인데
    // 실제 a_y 는 v_x*w_z 만큼 나오므로, 이 항이 없으면 정상 선회가 그대로
    // 고장으로 판정된다.  실캡처 3543 표본에서 잔차 중앙값 111 (임계 76),
    // 임계 초과 55.8 % 였고 구심항을 넣으면 중앙값 7 / 0.65 % 가 된다.
    //
    // 단위 : v_x LSB 0.01 m/s, w_z LSB 0.001 rad/s, a_y LSB 0.01 m/s^2 이므로
    //        정확한 계수는 1/1000 이다.  >>> LUT_SH (1/1024) 는 2.4 % 작지만
    //        그 오차는 최대 12 raw 로 임계 76 대비 무시할 수 있고 곱셈기를
    //        추가하지 않는다.
    logic signed [31:0] centripetal_pred;    // 관계식 4  : speed_x * gyro_z

    // 관계식 8 : yaw 는 +-18000 (+-180.00 deg) 에서 되감긴다.  단순 차분하면
    // 경계를 넘는 순간 차분이 36000 이 되어 기준값이 36000*C_GYR = 128,664,000
    // 까지 튄다 (실측 잔차 최대 128,662,714, 개루프 캡처에서 2회 관측).
    // 각도 차분은 원주상 최단 경로로 되감아야 물리적으로 옳다.
    // roll/pitch 는 +-30 deg 로 클램프되어 경계가 없으므로 그대로 둔다.
    logic signed [31:0] delta_incline_z_raw;
    logic signed [31:0] delta_incline_z_wrapped;
    logic signed [31:0] abs_y, abs_z;
    logic signed [31:0] abs_max, abs_min;
    logic signed [31:0] approx1, approx2, approx_sqrt;

    always_comb begin
        sin_x = get_sin(sim_data_in.incline_x);
        cos_x = get_cos(sim_data_in.incline_x);
        tan_x = get_tan(sim_data_in.incline_x);
        
        sin_y = get_sin(sim_data_in.incline_y);
        cos_y = get_cos(sim_data_in.incline_y);
        tan_y = get_tan(sim_data_in.incline_y);
        
        steer_lut_val = get_steer_lut(sim_data_in.steering);

        grav_x = -(G * sin_y) >>> LUT_SH;
        grav_y = (G * sin_x * cos_y) >>> (2 * LUT_SH);
        grav_z = (G * cos_x * cos_y) >>> (2 * LUT_SH);

        delta_incline_z_raw = sim_data_in.incline_z - sim_data_out.incline_z;
        if (delta_incline_z_raw > 32'sd18000)
            delta_incline_z_wrapped = delta_incline_z_raw - 32'sd36000;
        else if (delta_incline_z_raw < -32'sd18000)
            delta_incline_z_wrapped = delta_incline_z_raw + 32'sd36000;
        else
            delta_incline_z_wrapped = delta_incline_z_raw;

        steer_yaw_pred    = sim_data_in.speed_x * steer_lut_val;
        accel_y_tilt_pred = sensor_data_in.accel_z * tan_x;
        centripetal_pred  = sim_data_in.speed_x * sensor_data_in.gyro_z;

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
    end

    always_comb begin
        consistency_mask_1 = (abs(sim_data_in.incline_x) >= CLAMP_TH) | (abs(sim_data_in.incline_y) >= CLAMP_TH);
        consistency_mask_2 = (abs(sim_data_in.incline_x) > LVL_TH) | (abs(sim_data_in.incline_y) > LVL_TH) | (abs(sensor_data_in.gyro_z) > YAW_TH);
        consistency_mask_3 = (abs(sim_data_in.speed_x) < SPD_MIN) | (abs(sim_data_in.incline_x) >= CLAMP_TH) | (abs(sim_data_in.incline_y) >= CLAMP_TH);
    end
    
    //------------------------------------------------------------
    // Sequential Logic
    //------------------------------------------------------------

    logic signed [13:0] speed_x_prev [W];
    logic signed [13:0] speed_y_prev [W];
    logic signed [13:0] speed_z_prev [W];
    logic [7:0] win;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < W; i++) begin
                speed_x_prev[i] <= '0;
                speed_y_prev[i] <= '0;
                speed_z_prev[i] <= '0;
            end
        end 
        else if (valid_s0) begin
            speed_x_prev[0] <= sim_data_in.speed_x;
            speed_y_prev[0] <= sim_data_in.speed_y;
            speed_z_prev[0] <= sim_data_in.speed_z;
            
            for (int i = 1; i < W; i++) begin
                speed_x_prev[i] <= speed_x_prev[i-1];
                speed_y_prev[i] <= speed_y_prev[i-1];
                speed_z_prev[i] <= speed_z_prev[i-1];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pred_data_out <= '0;
        end
        else begin
            if (valid_s0) begin
                if (timeout_mask_1s || consistency_mask_1s_approach_speed || consistency_mask_20s_approach_speed || (win == M-1) || (sim_data_in.situation == 3'b001)) begin // 20s 추가
                    pred_data_out.pred_distance   <= sensor_data_in.distance * S_DIST;
                end
                else pred_data_out.pred_distance  <= pred_data_out.pred_distance - (sensor_data_out.approach_speed + sensor_data_in.approach_speed) * C_DIST;
                
                pred_data_out.pred_approach_speed <= -(({1'b0, sensor_data_in.distance}) - ({1'b0, sensor_data_out.distance})) * C_APSP;
                pred_data_out.pred_accel_x_1      <= (sim_data_in.speed_x - speed_x_prev[W-1]) * C_ACC + grav_x * S_ACC;
                pred_data_out.pred_accel_x_2      <= grav_x;
                pred_data_out.pred_accel_x_3      <= (-approx_sqrt * tan_y) >>> LUT_SH;
                pred_data_out.pred_accel_y_1      <= (sim_data_in.speed_y - speed_y_prev[W-1]) * C_ACC + grav_y * S_ACC + (centripetal_pred >>> LUT_SH);
                pred_data_out.pred_accel_y_2      <= grav_y;
                pred_data_out.pred_accel_y_3      <= accel_y_tilt_pred >>> LUT_SH;
                pred_data_out.pred_accel_z_1      <= (sim_data_in.speed_z - speed_z_prev[W-1]) * C_ACC + grav_z * S_ACC;
                pred_data_out.pred_accel_z_2      <= grav_z;
                pred_data_out.pred_gyro_x_1       <= (sim_data_in.incline_x - sim_data_out.incline_x) * C_GYR;
                pred_data_out.pred_gyro_x_2       <= '0;
                pred_data_out.pred_gyro_y_1       <= (sim_data_in.incline_y - sim_data_out.incline_y) * C_GYR;
                pred_data_out.pred_gyro_y_2       <= '0;
                pred_data_out.pred_gyro_z_1       <= delta_incline_z_wrapped * C_GYR;
                pred_data_out.pred_gyro_z_2       <= '0;
                pred_data_out.pred_gyro_z_3       <= steer_yaw_pred >>> LUT_SH;
            end
        end
    end

    always_ff @(posedge clk) begin

        if(!rst_n) begin
            sensor_data_out         <= '0;
            sim_data_out            <= '0;
            prev_sensor_data_out    <= '0;
            processed_data_out      <= '0;
            prev_processed_data_out <= '0;
            valid_s1                <= 1'b0;
            sample_seq_out          <= '0;
            clk_cnt                 <= '0;
            win                     <= '0; 
        end
        else begin
            //----------------------------------------------------
            // Update Only When New Sample Arrives
            //----------------------------------------------------

            if(valid_s0) begin
                sensor_data_out         <= sensor_data_in;
                sim_data_out            <= sim_data_in;
                prev_sensor_data_out    <= sensor_data_out;
                processed_data_out      <= processed_data;
                prev_processed_data_out <= prev_processed_data;
                valid_s1                <= valid_s0;
                sample_seq_out          <= sample_seq_in;
                clk_cnt                 <= '0;
                if (timeout_mask_1s || consistency_mask_1s_approach_speed || consistency_mask_20s_approach_speed || (win == M-1) || (sim_data_in.situation == 3'b001)) //20s추가
                    win <= '0;
                else 
                    win <= win + 1;
            end

            //----------------------------------------------------
            // No New Sample
            //----------------------------------------------------

            else begin
                valid_s1 <= valid_s0;
                clk_cnt <= clk_cnt + 32'd1;
            end
        end
    end

endmodule
