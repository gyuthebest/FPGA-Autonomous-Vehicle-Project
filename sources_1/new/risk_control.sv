import types_pkg::*;

module risk_control_2 #(
    parameter int CLK_FREQ = 100000000   // 100MHz 가정
) (
    input logic clk,
    input logic rst_n,

    input logic valid_in,
    input logic valid_in_rel,
    input logic [31:0] sample_seq_in,
    input sensor_data_t sensor_data_in,
    input sim_data_t sim_data_in,


    input reliability_state_t rel_in,
    input risk_t risk_in,

   
    

    output logic [31:0] sample_seq_out_risk,
    //output logic valid_out_risk,
    //output logic valid_out_rel,

    output logic [1:0] valid_out_rel_risk,
    
    output risk_t risk_out,
    output reliability_state_t rel_out,

    output sensor_data_t sensor_data_out,
    output sim_data_t sim_data_out,

    output logic transition_demand,
    output logic hud_warning,
    output logic mrm,
    output logic [3:0] td_remain_sec
    

);

    localparam logic [1:0] NORMAL   = 2'b00;
    localparam logic [1:0] DEGRADED = 2'b01;
    localparam logic [1:0] INVALID  = 2'b10;

    // 조합해서 만드는 위험도 요소에 관한 신뢰도
    logic [1:0] Re_collision;
    logic [1:0] Re_road_A; // surface
    logic [1:0] Re_road_B;  // impact
    logic [1:0] Re_vision_A; // 온습도
    logic [1:0] Re_posture_A; //roll
    logic [1:0] Re_posture_B; //yaw
    logic [1:0] Re_posture_C; //횡가속

    logic [1:0] Re_pitch;
    logic [1:0] Re_longitudinal;

    // 위험도 요소별 최종 위험도
    logic [2:0] eff_tier_collision;
    logic [1:0] eff_tier_road_A;
    logic [1:0] eff_tier_road_B;
    logic [1:0] eff_tier_vision_A;
    logic [1:0] eff_tier_vision_B;
    logic eff_tier_posture_A;
    logic [1:0] eff_tier_posture_B;
    logic [1:0] eff_tier_posture_C;

    // 신뢰도 invalid 되기 직전 값 저장
    logic [2:0] last_valid_Ri_collision;
    logic [1:0] last_valid_Ri_road_A;
    logic [1:0] last_valid_Ri_road_B;
    logic [1:0] last_valid_Ri_vision_A;
    logic last_valid_Ri_posture_A;
    logic [1:0] last_valid_Ri_posture_B;
    logic [1:0] last_valid_Ri_posture_C;

    logic [12:0] road_A_speed_limit;
    logic [12:0] road_B_speed_limit;
    logic [12:0] vision_A_speed_limit;
    logic [12:0] vision_B_speed_limit;


    logic making_col_inv, making_col_deg;
    logic making_rs_inv, making_rs_deg;
    logic making_ri_inv, making_ri_deg;
    logic making_vi_inv, making_vi_deg;
    logic making_roll_inv, making_roll_deg;
    logic making_yaw_inv, making_yaw_deg;
    logic making_accy_inv, making_accy_deg;
    logic making_pitch_inv, making_pitch_deg;
    logic making_long_inv, making_long_deg;

    assign valid_out_rel_risk[1] = valid_out_rel;
    assign valid_out_rel_risk[0] = valid_out_risk;

    // 위험도 요소별 신뢰도 조합
    always_comb begin
        // Collision
        making_col_inv = (rel_in.distance.state == INVALID) || (rel_in.approach_speed.state == INVALID);
        making_col_deg = (rel_in.distance.state == DEGRADED) || (rel_in.approach_speed.state == DEGRADED);

        if (making_col_inv) 
            Re_collision = INVALID;
        else if (making_col_deg) 
            Re_collision = DEGRADED;
        else 
            Re_collision = NORMAL;


        // Road Surface
        making_rs_inv = (rel_in.temperature.state == INVALID) || (rel_in.humidity.state == INVALID);
        making_rs_deg = (rel_in.temperature.state == DEGRADED) || (rel_in.humidity.state == DEGRADED);

        if (making_rs_inv)
            Re_road_A = INVALID;
        else if (making_rs_deg)
            Re_road_A = DEGRADED;
        else 
            Re_road_A = NORMAL;

        // Road Impact
        making_ri_inv = (rel_in.accel_z.state == INVALID);
        making_ri_deg = (rel_in.accel_z.state == DEGRADED);

        if (making_ri_inv) 
            Re_road_B = INVALID;
        else if (making_ri_deg) 
            Re_road_B = DEGRADED;
        else 
            Re_road_B = NORMAL;


        // Vision_A
        making_vi_inv = (rel_in.lux.state == INVALID);
        making_vi_deg = (rel_in.lux.state == DEGRADED);

        if (making_vi_inv) 
            Re_vision_A = INVALID;
        else if (making_vi_deg) 
            Re_vision_A = DEGRADED;
        else 
            Re_vision_A = NORMAL;

        // Roll
        making_roll_inv = (rel_in.gyro_x.state == INVALID);
        making_roll_deg = (rel_in.gyro_x.state == DEGRADED);

        if (making_roll_inv) 
            Re_posture_A = INVALID;
        else if (making_roll_deg) 
            Re_posture_A = DEGRADED;
        else 
            Re_posture_A = NORMAL;


        // Yaw
        making_yaw_inv = (rel_in.gyro_z.state == INVALID);
        making_yaw_deg = (rel_in.gyro_z.state == DEGRADED);

        if (making_yaw_inv) 
            Re_posture_B = INVALID;
        else if (making_yaw_deg) 
            Re_posture_B = DEGRADED;
        else 
            Re_posture_B = NORMAL;

        // Lateral(accel_y)
        making_accy_inv = (rel_in.accel_y.state == INVALID);
        making_accy_deg = (rel_in.accel_y.state == DEGRADED);

        if (making_accy_inv) 
            Re_posture_C = INVALID;
        else if (making_accy_deg) 
            Re_posture_C = DEGRADED;
        else 
            Re_posture_C = NORMAL;


        // 제어 미사용, 경고에만 사용: Pitch
        making_pitch_inv = (rel_in.gyro_y.state == INVALID);
        making_pitch_deg = (rel_in.gyro_y.state == DEGRADED);

        if (making_pitch_inv) 
            Re_pitch = INVALID;
        else if (making_pitch_deg) 
            Re_pitch = DEGRADED;
        else 
            Re_pitch = NORMAL;

        // 제어 미사용, 경고에만 사용: Longitudinal
        making_long_inv = (rel_in.accel_x.state == INVALID);
        making_long_deg = (rel_in.accel_x.state == DEGRADED);

        if (making_long_inv) 
            Re_longitudinal = INVALID;
        else if (making_long_deg) 
            Re_longitudinal = DEGRADED;
        else 
            Re_longitudinal = NORMAL;
    end




    //effective_tier 결정
    function automatic logic [2:0] calc_effective_tier(
        input logic [2:0] raw_risk_tier,
        input logic [2:0] last_valid_tier,
        input logic [1:0] reliability_state,
        input int N // 위험도 state 분류 몇개인지
    );
        int pre, last_deg, floor_tier, effective_tier;

        if (reliability_state == 2'b00) // NORMAL
            effective_tier = raw_risk_tier;
        else if (reliability_state == 2'b01) begin//DEGRADED
            pre = (raw_risk_tier +1 < N-2) ? raw_risk_tier + 1 : (N-2);
            effective_tier = (raw_risk_tier < pre) ? pre : raw_risk_tier;
        end
        else begin // INVALID

            case (N)
                2: floor_tier = 1;
                3: floor_tier = 1;
                4: floor_tier = 2;
                5: floor_tier = 2;
            endcase

            

            pre = (last_valid_tier +1 < N-2) ? last_valid_tier + 1 : (N-2);
            last_deg = (last_valid_tier < pre) ? pre : last_valid_tier; 
            effective_tier = (last_deg < floor_tier) ? floor_tier : last_deg;
        end

        return effective_tier[2:0];
    endfunction

    
    always_comb begin
        eff_tier_collision = calc_effective_tier(risk_in.Ri_collision, last_valid_Ri_collision, Re_collision, 5);
        eff_tier_road_A = calc_effective_tier({1'b0, risk_in.Ri_road_A}, {1'b0, last_valid_Ri_road_A}, Re_road_A, 4);
        eff_tier_road_B = calc_effective_tier({1'b0, risk_in.Ri_road_B}, {1'b0, last_valid_Ri_road_B}, Re_road_B, 4);
        eff_tier_vision_A = calc_effective_tier({1'b0, risk_in.Ri_vision_A}, {1'b0, last_valid_Ri_vision_A}, Re_vision_A, 4);
        eff_tier_vision_B = risk_in.Ri_vision_B;
        eff_tier_posture_A = calc_effective_tier({2'b00, risk_in.Ri_posture_A}, {2'b00, last_valid_Ri_posture_A}, Re_posture_A, 2);
        eff_tier_posture_B = calc_effective_tier({1'b0, risk_in.Ri_posture_B}, {1'b0, last_valid_Ri_posture_B}, Re_posture_B, 3);
        eff_tier_posture_C = calc_effective_tier({1'b0, risk_in.Ri_posture_C}, {1'b0, last_valid_Ri_posture_C}, Re_posture_C, 3);
    end



    
    // 제어 
    logic [3:0] col_accelerator, road_A_accelerator, road_B_accelerator, vision_A_accelerator, vision_B_accelerator, posture_A_accelerator, posture_B_accelerator, posture_C_accelerator;
    logic [3:0] col_brake, road_B_brake;
    logic [1:0] col_gear;
    logic signed [7:0] posture_A_steering, posture_B_steering, posture_C_steering;
    logic col_hazard, vision_A_headlight, vision_B_headlight, vision_B_hazard;

    logic [3:0] final_accelerator;
    logic [3:0] final_brake;
    logic [1:0] final_gear;
    logic signed [7:0] final_steering;
    logic [12:0] final_speed_limit;
    logic final_headlight;
    logic final_hazard;

    logic can_downshift;
    
    localparam int COUNT_05_FREQ = CLK_FREQ >>> 1;
    localparam int CNT_WIDTH = (COUNT_05_FREQ <= 1) ? 1: $clog2(COUNT_05_FREQ + 1);

    logic [CNT_WIDTH - 1:0] cnt_05;

    

    logic [12:0] spd_limit_90;
    logic [12:0] spd_limit_80;
    logic [12:0] spd_limit_70;
    logic [12:0] spd_limit_60;
    logic [12:0] spd_limit_50;

    always_comb begin

        // Gemini의 최적화: 소수점 연산을 1024(2^10) 기반 비트 시프트 정수 연산으로 변환 ---
        // 0.9 ≒ 922/1024, 0.8 ≒ 819/1024, 0.7 ≒ 717/1024, 0.6 ≒ 614/1024, 0.5 = >> 1
        spd_limit_90 = (24'(sim_data_in.speed_limit) * 922) >> 10;
        spd_limit_80 = (24'(sim_data_in.speed_limit) * 819) >> 10;
        spd_limit_70 = (24'(sim_data_in.speed_limit) * 717) >> 10;
        spd_limit_60 = (24'(sim_data_in.speed_limit) * 614) >> 10;
        spd_limit_50 = sim_data_in.speed_limit >> 1;

        col_accelerator = sim_data_in.accelerator;
        road_A_accelerator = sim_data_in.accelerator;
        road_B_accelerator = sim_data_in.accelerator;
        vision_A_accelerator = sim_data_in.accelerator;
        vision_B_accelerator = sim_data_in.accelerator;
        posture_A_accelerator = sim_data_in.accelerator;
        posture_B_accelerator = sim_data_in.accelerator;
        posture_C_accelerator = sim_data_in.accelerator;

        col_brake = sim_data_in.brake;
        road_B_brake = sim_data_in.brake;
        col_gear = sim_data_in.gear;

        posture_A_steering = sim_data_in.steering;
        posture_B_steering = sim_data_in.steering;
        posture_C_steering = sim_data_in.steering;

        col_hazard = 1'b0;
        vision_B_hazard = 1'b0;
        vision_A_headlight = 1'b0;
        vision_B_headlight = 1'b0;

        road_A_speed_limit = sim_data_in.speed_limit;
        road_B_speed_limit = sim_data_in.speed_limit;
        vision_A_speed_limit = sim_data_in.speed_limit;
        vision_B_speed_limit = sim_data_in.speed_limit;

        // 충돌 위험
        case (eff_tier_collision)
            3'b001: col_accelerator = 4'd0;
            3'b010: begin 
                col_accelerator = 4'd0;
                if (sim_data_in.speed_x <= 14'sd1111) // 40 km/h
                    col_brake = 4'd2;
                else if (sim_data_in.speed_x <= 14'sd2222) // 80 km/h 
                    col_brake = 4'd3;
                else
                    col_brake = 4'd4;
                
                if (sim_data_in.rpm <= 2'd1 && can_downshift && sim_data_in.gear > 2'd0)
                    col_gear = sim_data_in.gear - 2'd1;
            end

            3'b011: begin
                col_accelerator = 4'd0;

                if (sim_data_in.speed_x <= 14'sd1111)
                    col_brake = 4'd4;
                else if (sim_data_in.speed_x <= 14'sd2222)
                    col_brake = 4'd6;
                else                 
                    col_brake = 4'd8;

                if (sim_data_in.rpm <= 2'd1 && can_downshift && sim_data_in.gear > 2'd0)
                    col_gear = sim_data_in.gear - 2'd1;

                col_hazard = 1'b1;
            end

            3'b100: begin
                col_accelerator = 4'd0;

                col_brake = 4'd10;

                if (sim_data_in.rpm <= 2'd1 && can_downshift && sim_data_in.gear > 2'd0)
                    col_gear = sim_data_in.gear - 2'd1;

                col_hazard = 1'b1;
            end

            default: ;
        endcase

        //노면 위험
        case (eff_tier_road_A)
            2'b01: begin
                if (sim_data_in.accelerator > 4'd8) 
                    road_A_accelerator = 4'd8;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_90}))) 
                    road_A_accelerator = 4'd0;
                road_A_speed_limit = spd_limit_90;
            end

            2'b10: begin
                if (sim_data_in.accelerator > 4'd6) 
                    road_A_accelerator = 4'd6;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_70})))
                    road_A_accelerator = 4'd0;
                road_A_speed_limit = spd_limit_70;
            end

            2'b11: begin
                if (sim_data_in.accelerator > 4'd4) 
                    road_A_accelerator = 4'd4;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_50})))
                    road_A_accelerator = 4'd0;
                road_A_speed_limit = spd_limit_50;
            end

            default: ;
        endcase


        case (eff_tier_road_B)
            2'b01: begin
                if (sim_data_in.accelerator > 4'd9) 
                    road_B_accelerator = 4'd9;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_80})))
                    road_B_accelerator = 4'd0;
                road_B_brake = 4'd2;
                road_B_speed_limit = spd_limit_80;
            end

            2'b10: begin
                if (sim_data_in.accelerator > 4'd7) 
                    road_B_accelerator = 4'd7;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_60})))
                    road_B_accelerator = 4'd0;
                road_B_brake = 4'd2;
                road_B_speed_limit = spd_limit_60;
            end

            2'b11: begin
                if (sim_data_in.accelerator > 4'd5) 
                    road_B_accelerator = 4'd5;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_50})))
                    road_B_accelerator = 4'd0;
                road_B_brake = 4'd2;
                road_B_speed_limit = spd_limit_50;
            end


            default: ;
        endcase


        //시야 위험
        case (eff_tier_vision_A)
            2'b01, 2'b10: begin
                vision_A_headlight = 1'b1;
            end

            2'b11: begin
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_90}))) 
                    vision_A_accelerator = 4'd0;
                vision_A_headlight = 1'b1;
                vision_A_speed_limit = spd_limit_90;
            end

            default: ;
        endcase

        case (eff_tier_vision_B)
            2'b01: begin
                if (sim_data_in.accelerator > 4'd8) 
                    vision_B_accelerator = 4'd8;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_90}))) 
                    vision_B_accelerator = 4'd0; 
                vision_B_speed_limit = spd_limit_90; 
                vision_B_headlight = 1'b1;
            end
            2'b10: begin
                if (sim_data_in.accelerator > 4'd8) 
                    vision_B_accelerator = 4'd8;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_70})))
                    vision_B_accelerator = 4'd0; 
                vision_B_speed_limit = spd_limit_70; 
                vision_B_headlight = 1'b1;
                vision_B_hazard = 1'b1;
            end
            2'b11: begin
                if (sim_data_in.accelerator > 4'd5) 
                    vision_B_accelerator = 4'd5;
                if ((sim_data_in.speed_x > 0) && (sim_data_in.speed_x > $signed({1'b0, spd_limit_60})))
                    vision_B_accelerator = 4'd0;
                vision_B_speed_limit = spd_limit_60;
                vision_B_headlight = 1'b1;
            end
            default: ;
        endcase

        //자세 위험
        case (eff_tier_posture_A)
            1'b1: begin
                posture_A_accelerator = 4'd0;

                if (sim_data_in.delta_steering > 100)
                    posture_A_steering = sim_data_out.steering + 100;
                else if(sim_data_in.delta_steering < -100)
                    posture_A_steering = sim_data_out.steering - 100;
            end

            default: ;
        endcase

        case (eff_tier_posture_B)
            2'b01: begin
                if (sim_data_in.accelerator > 4'd8)
                    posture_B_accelerator = 4'd8;

                if (sim_data_in.delta_steering > 140)
                    posture_B_steering = sim_data_out.steering + 140;
                else if(sim_data_in.delta_steering < -140)
                    posture_B_steering = sim_data_out.steering - 140;
            end

            2'b10: begin
                    posture_B_accelerator = 4'd0;

                if (sim_data_in.delta_steering > 100)
                    posture_B_steering = sim_data_out.steering + 100;
                else if(sim_data_in.delta_steering < -100)
                    posture_B_steering = sim_data_out.steering - 100;
            end

            default: ;
        endcase

        case (eff_tier_posture_C)
            2'b01: begin
                if (sim_data_in.accelerator > 4'd7)
                    posture_C_accelerator = 4'd7;

                if (sim_data_in.delta_steering > 160)
                    posture_C_steering = sim_data_out.steering + 160;
                else if(sim_data_in.delta_steering < -160)
                    posture_C_steering = sim_data_out.steering - 160;
            end

            2'b10: begin
                    posture_C_accelerator = 4'd0;

                if (sim_data_in.delta_steering > 120)
                    posture_C_steering = sim_data_out.steering + 120;
                else if(sim_data_in.delta_steering < -120)
                    posture_C_steering = sim_data_out.steering - 120;
            end

            default: ;
        endcase
    end


    //최종 제어

    function automatic logic [3:0] get_min4(input logic [3:0] a, input logic [3:0] b); return (a < b) ? a : b; endfunction
    function automatic logic [3:0] get_max4(input logic [3:0] a, input logic [3:0] b); return (a > b) ? a : b; endfunction
    function automatic logic [1:0] get_min2(input logic [1:0] a, input logic [1:0] b); return (a < b) ? a : b; endfunction
    function automatic logic [12:0] get_min13(input logic [12:0] a, input logic [12:0] b); return (a < b) ? a : b; endfunction

    function automatic logic [8:0] abs_val(input logic signed [8:0] val); return (val >= 0) ? val : -val; endfunction


    logic [3:0] min_acc1, min_acc2, min_acc3, min_acc4;
    logic dangerous_brake_situation;
    logic signed [8:0] diff_in, diff_A, diff_B, diff_C;
    logic signed [8:0] min_diff_1, min_diff_2;
    logic signed [7:0] min_steer_1, min_steer_2;
    logic [12:0] min_spd1, min_spd2;

    always_comb begin
        // Accelerator
        min_acc1 = get_min4(col_accelerator, road_A_accelerator);
        min_acc2 = get_min4(road_B_accelerator, vision_A_accelerator);
        min_acc3 = get_min4(vision_B_accelerator, posture_A_accelerator);
        min_acc4 = get_min4(posture_B_accelerator, posture_C_accelerator);
        final_accelerator = get_min4(get_min4(min_acc1, min_acc2), get_min4(min_acc3, min_acc4));

        // Brake
        dangerous_brake_situation = (eff_tier_road_A >= 2'b10) || (eff_tier_posture_C >= 2'b10);

        if (dangerous_brake_situation)
            final_brake = 4'd0;
        else 
            final_brake = get_max4(col_brake, road_B_brake);
        
        // Steering
        diff_in = abs_val(sim_data_in.steering - sim_data_out.steering);
        diff_A = abs_val(posture_A_steering - sim_data_out.steering);
        diff_B = abs_val(posture_B_steering - sim_data_out.steering);
        diff_C = abs_val(posture_C_steering - sim_data_out.steering);

        // final_steering
        if (diff_A <= diff_B) begin 
            min_diff_1 = diff_A; 
            min_steer_1 = posture_A_steering; 
        end
        else begin 
            min_diff_1 = diff_B; 
            min_steer_1 = posture_B_steering; 
        end

        if (diff_C <= diff_in) begin 
            min_diff_2 = diff_C; 
            min_steer_2 = posture_C_steering; 
        end
        else begin 
            min_diff_2 = diff_in; 
            min_steer_2 = sim_data_in.steering; 
        end

        
        if (min_diff_1 <= min_diff_2) 
            final_steering = min_steer_1;
        else                          
            final_steering = min_steer_2;

        // Speed Limit 최솟값 도출
        min_spd1 = get_min13(road_A_speed_limit, road_B_speed_limit);
        min_spd2 = get_min13(vision_A_speed_limit, vision_B_speed_limit);
        final_speed_limit = get_min13(min_spd1, min_spd2);

        // Gear, Headlight, Hazard
        final_gear = get_min2(col_gear, sim_data_in.gear);
        final_headlight = vision_A_headlight | vision_B_headlight;
        final_hazard = col_hazard | vision_B_hazard;

        // MRM 상황
        if (mrm) begin
            final_accelerator = 4'd0;
            final_brake = 4'd3;
            final_hazard = 1'b1;
            if (sim_data_in.rpm <= 2'd1 && can_downshift && sim_data_in.gear > 2'd0) begin
                final_gear = final_gear - 2'd1;
            end
        end
    end

    //경고창, 타이머, TD
    logic td_condition;

    always_comb begin
        hud_warning = (Re_collision == INVALID) || (Re_road_A == INVALID) || 
                      (Re_road_B == INVALID) || (Re_vision_A == INVALID) || 
                      (Re_posture_A == INVALID) || (Re_posture_B == INVALID) || 
                      (Re_posture_C == INVALID) || (Re_pitch == INVALID) || 
                      (Re_longitudinal == INVALID);
        
        // TD 발동 조건: 축소운행 불가 그룹 중 하나라도 INVALID
        td_condition = (Re_collision == INVALID) || (Re_posture_A == INVALID) || 
                       (Re_posture_B == INVALID) || (Re_posture_C == INVALID);
        
        mrm = (td_remain_sec == 4'd0);
        
        transition_demand = (td_remain_sec <= 4'd10);
    end

    
    // 타이머, 카운트다운
    localparam int ONE_SEC_CYCLES = CLK_FREQ; // 1초 카운트용 (100MHz 기준)
    logic [$clog2(ONE_SEC_CYCLES)-1:0] sec_cnt;
    
    logic [2:0] td_invalid_duration;
    logic td_locked;        

    // -------------------------------------------------------------
    // 1초 단위 타이머 및 TD 로직
    // -------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sec_cnt <= '0;
            td_invalid_duration <= '0;
            td_locked <= 1'b0;
            td_remain_sec <= 4'd11; // 파이썬에서 '-'로 표시할 초기값
        end 
        else if (sim_data_in.manual_mode == 1'b1) begin
            sec_cnt <= '0;
            td_invalid_duration <= '0;
            td_locked <= 1'b0;
            td_remain_sec <= 4'd11;
        end 
        else begin
           
            if (sec_cnt >= ONE_SEC_CYCLES - 1) begin
                sec_cnt <= '0;
                // INVALID 5초 유지 조건 검사
                if (td_condition) begin
                    if (td_invalid_duration < 3'd5)
                        td_invalid_duration <= td_invalid_duration + 3'd1;
                    
                    if (td_invalid_duration >= 3'd4)
                        td_locked <= 1'b1;
                end 
                else begin
                    if (!td_locked) begin
                        td_invalid_duration <= '0;
                    end
                end
                // 10초 카운트다운 로직
                if (td_condition || td_locked) begin
                    if (td_remain_sec == 4'd11) begin
                        td_remain_sec <= 4'd10;      // 11에서 10으로 진입
                    end 
                    else if (td_remain_sec > 4'd0)
                        td_remain_sec <= td_remain_sec - 4'd1; // 1초에 1씩 감소
                end 
                else begin
                    if (!td_locked) begin
                        td_remain_sec <= 4'd11;
                    end
                end
            end 
            else begin
                sec_cnt <= sec_cnt + 1;
            end
        end
    end
    // -------------------------------------------------------------
    // 데이터 갱신 및 제어 로직
    // -------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sim_data_out.accelerator <= '0;
            sim_data_out.brake <= 4'd5;
            sim_data_out.steering <= '0;
            sim_data_out.gear <= 2'd0;
            sim_data_out.headlight <= 1'b0;
            sim_data_out.hazard <= 1'b0;
            sim_data_out.manual_mode <= sim_data_in.manual_mode; 
            sim_data_out.speed_limit <= 13'b0; 
            sample_seq_out_risk <= '0;
            
            // valid 초기화
            valid_out_risk <= 1'b0;
            valid_out_rel <= 1'b0;
            risk_out.Ri_collision <= '0;
            risk_out.Ri_road_A <= '0;
            risk_out.Ri_road_B <= '0;
            risk_out.Ri_vision_A <= '0;
            risk_out.Ri_vision_B <= '0;
            risk_out.Ri_posture_A <= '0;
            risk_out.Ri_posture_B <= '0;
            risk_out.Ri_posture_C <= '0;
            last_valid_Ri_collision <= NORMAL;
            last_valid_Ri_road_A <= NORMAL;
            last_valid_Ri_road_B <= NORMAL;
            last_valid_Ri_vision_A <= NORMAL;
            last_valid_Ri_posture_A <= NORMAL;
            last_valid_Ri_posture_B <= NORMAL;
            last_valid_Ri_posture_C <= NORMAL;
            cnt_05 <= COUNT_05_FREQ;
            can_downshift <= 1'b1;
        end
        else begin
            
            valid_out_risk <= valid_in;
            valid_out_rel  <= valid_in_rel;
            
            if (valid_in) begin
                sample_seq_out_risk <= sample_seq_in;
                
                if (sim_data_in.manual_mode) begin
                    sim_data_out <= sim_data_in;
                    sim_data_out.headlight <= final_headlight;
                    sim_data_out.hazard <= final_hazard;
                    
                    risk_out.Ri_collision <= eff_tier_collision;
                    risk_out.Ri_road_A <= eff_tier_road_A;
                    risk_out.Ri_road_B <= eff_tier_road_B;
                    risk_out.Ri_vision_A <= eff_tier_vision_A;
                    risk_out.Ri_vision_B <= eff_tier_vision_B;
                    risk_out.Ri_posture_A <= eff_tier_posture_A;
                    risk_out.Ri_posture_B <= eff_tier_posture_B;
                    risk_out.Ri_posture_C <= eff_tier_posture_C;
                    if(Re_collision != INVALID) last_valid_Ri_collision <= eff_tier_collision;
                    if(Re_road_A != INVALID)    last_valid_Ri_road_A <= eff_tier_road_A;
                    if(Re_road_B != INVALID)    last_valid_Ri_road_B <= eff_tier_road_B;
                    if(Re_vision_A != INVALID)  last_valid_Ri_vision_A <= eff_tier_vision_A;
                    if(Re_posture_A != INVALID) last_valid_Ri_posture_A <= eff_tier_posture_A;
                    if(Re_posture_B != INVALID) last_valid_Ri_posture_B <= eff_tier_posture_B;
                    if(Re_posture_C != INVALID) last_valid_Ri_posture_C <= eff_tier_posture_C;
                end
                else begin 
                    sim_data_out.accelerator <= final_accelerator;
                    sim_data_out.brake <= final_brake;
                    sim_data_out.steering <= final_steering;
                    sim_data_out.gear <= final_gear;
                    sim_data_out.headlight <= final_headlight;
                    sim_data_out.hazard <= final_hazard;
                    sim_data_out.manual_mode <= sim_data_in.manual_mode;
                    sim_data_out.speed_limit <= final_speed_limit;
                    cnt_05 <= COUNT_05_FREQ;
                    risk_out.Ri_collision <= eff_tier_collision;
                    risk_out.Ri_road_A <= eff_tier_road_A;
                    risk_out.Ri_road_B <= eff_tier_road_B;
                    risk_out.Ri_vision_A <= eff_tier_vision_A;
                    risk_out.Ri_vision_B <= eff_tier_vision_B;
                    risk_out.Ri_posture_A <= eff_tier_posture_A;
                    risk_out.Ri_posture_B <= eff_tier_posture_B;
                    risk_out.Ri_posture_C <= eff_tier_posture_C;
                    if(Re_collision != INVALID) last_valid_Ri_collision <= eff_tier_collision;
                    if(Re_road_A != INVALID)    last_valid_Ri_road_A <= eff_tier_road_A;
                    if(Re_road_B != INVALID)    last_valid_Ri_road_B <= eff_tier_road_B;
                    if(Re_vision_A != INVALID)  last_valid_Ri_vision_A <= eff_tier_vision_A;
                    if(Re_posture_A != INVALID) last_valid_Ri_posture_A <= eff_tier_posture_A;
                    if(Re_posture_B != INVALID) last_valid_Ri_posture_B <= eff_tier_posture_B;
                    if(Re_posture_C != INVALID) last_valid_Ri_posture_C <= eff_tier_posture_C;
                end
            end
        end
    end
endmodule
