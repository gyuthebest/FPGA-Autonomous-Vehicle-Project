import types_pkg::*;

module Driving_based_on_Risk #(
    parameter clk_freq = 100000000 // 100MHz
)(
    input logic clk,
    input logic rst_n,
    input logic is_trash,           
    input logic [31:0] sample_seq_s1,  

    input sensor_data_t sensor_data_in,
    input processed_data_t processed_data_in, // Z축 가속도 변화량(17-bit) 수신용
    
    output sensor_data_t sensor_data_out,
    output logic [4:0] steering_limit_out,    // Rate Limit 방출 포트
    output logic [31:0] sample_seq_s
);

    // =========================================================================
    // 물리량 임계값 상수 선언 (Localparams)
    // =========================================================================
    // 가속도: 1.0g = 1000 기준 (17-bit로 선언하여 delta_accel_z와 크기 매칭)
    localparam logic [16:0] G_0_5 = 17'd500;
    localparam logic [16:0] G_0_8 = 17'd800;
    localparam logic [16:0] G_1_0 = 17'd1000;
    localparam logic [16:0] G_2_0 = 17'd2000;

    // 각속도: 1.0 rad/s = 1000 스케일 (16-bit)
    localparam logic [15:0] GYRO_30 = 16'd524;
    localparam logic [15:0] GYRO_40 = 16'd698;
    localparam logic [15:0] GYRO_60 = 16'd1047;

    // 내부 위험도 레지스터
    logic [2:0] collision_risk;
    logic [1:0] road_risk_A;
    logic [1:0] road_risk_B;
    logic [1:0] vision_risk_A;
    logic [1:0] vision_risk_B;
    logic posture_risk_A;
    logic [1:0] posture_risk_B;
    logic [1:0] posture_risk_C;

    // 이전 상태 추적용 레지스터 (순차 회로 기어 제어용)
    logic [2:0] prev_collision_risk;

    // 절댓값 연산 결과 저장 레지스터
    logic [15:0] abs_gyro_x, abs_gyro_z, abs_accel_y;
    logic [16:0] abs_delta_accel_z;

    // 기어 2-Stage 강하 타이머 및 횟수 추적 레지스터
    logic [$clog2(clk_freq / 2 + 1)-1:0] gear_cooldown_cnt;
    logic [1:0] shift_count;
    logic [1:0] max_allowed_shifts;
    logic can_downshift;

    // =========================================================================
    // 1. 센서 입력 절댓값 변환
    // =========================================================================
    always_comb begin
        if (sensor_data_in.gyro_x[15] == 1'b1) begin abs_gyro_x = ~(sensor_data_in.gyro_x) + 16'd1; end 
        else begin abs_gyro_x = sensor_data_in.gyro_x; end
        
        if (sensor_data_in.gyro_z[15] == 1'b1) begin abs_gyro_z = ~(sensor_data_in.gyro_z) + 16'd1; end 
        else begin abs_gyro_z = sensor_data_in.gyro_z; end
        
        if (sensor_data_in.accel_y[15] == 1'b1) begin abs_accel_y = ~(sensor_data_in.accel_y) + 16'd1; end 
        else begin abs_accel_y = sensor_data_in.accel_y; end
        
        if (processed_data_in.delta_accel_z[16] == 1'b1) begin abs_delta_accel_z = ~(processed_data_in.delta_accel_z) + 17'd1; end 
        else begin abs_delta_accel_z = processed_data_in.delta_accel_z; end
    end

    // =========================================================================
    // 2. 위험도 판단부 (Risk Assessment)
    // =========================================================================
    always_comb begin
        // --- 충돌 위험 ---
        if (sensor_data_in.app_speed <= 0) begin collision_risk = 3'b000; end 
        else begin
            if (sensor_data_in.distance <= (14 * sensor_data_in.app_speed)) begin collision_risk = 3'b100; end // Emergency
            else if (sensor_data_in.distance <= (20 * sensor_data_in.app_speed)) begin collision_risk = 3'b011; end // Critical
            else if (sensor_data_in.distance <= (30 * sensor_data_in.app_speed)) begin collision_risk = 3'b010; end // Danger
            else if (sensor_data_in.distance <= (40 * sensor_data_in.app_speed)) begin collision_risk = 3'b001; end // Caution
            else begin collision_risk = 3'b000; end // Safe
        end

        // --- 노면 위험 ---
        if (sensor_data_in.temperature <= -50 && sensor_data_in.humidity >= 90) begin road_risk_A = 2'b11; end // Black Ice (-5.0도)
        else if (sensor_data_in.temperature <= 0 && sensor_data_in.humidity >= 70) begin road_risk_A = 2'b10; end // Ice (0.0도)
        else if (sensor_data_in.humidity >= 70) begin road_risk_A = 2'b01; end // Wet
        else begin road_risk_A = 2'b00; end // Dry

        if (sensor_data_in.speed < 30) begin road_risk_B = 2'b00; end 
        else begin 
            if (abs_delta_accel_z >= G_2_0) begin road_risk_B = 2'b11; end // 극심한 충격
            else if (abs_delta_accel_z >= G_1_0) begin road_risk_B = 2'b10; end // 심한 충격
            else if (abs_delta_accel_z >= G_0_5) begin road_risk_B = 2'b01; end // 거친 노면
            else begin road_risk_B = 2'b00; end
        end

        // --- 시야 위험 ---
        if (sensor_data_in.lux >= 20000) begin vision_risk_A = 2'b00; end
        else if (sensor_data_in.lux >= 1000) begin vision_risk_A = 2'b01; end
        else if (sensor_data_in.lux >= 50) begin vision_risk_A = 2'b10; end
        else begin vision_risk_A = 2'b11; end

        vision_risk_B = sensor_data_in.weather;

        // --- 자세 위험 ---
        if (abs_gyro_x > GYRO_40) begin posture_risk_A = 1'b1; end // 위험
        else begin posture_risk_A = 1'b0; end // 안전

        if (abs_gyro_z >= GYRO_60) begin posture_risk_B = 2'b10; end // 위험
        else if (abs_gyro_z >= GYRO_30) begin posture_risk_B = 2'b01; end // 주의
        else begin posture_risk_B = 2'b00; end // 정상

        if (abs_accel_y >= G_0_8) begin posture_risk_C = 2'b10; end // 위험
        else if (abs_accel_y >= G_0_5) begin posture_risk_C = 2'b01; end // 주의
        else begin posture_risk_C = 2'b00; end // 정상
    end

    // =========================================================================
    // 3. 기어 강하 상태 추적기 (Downshift State Tracker)
    // =========================================================================
    always_comb begin
        if (collision_risk >= 3'b011) begin max_allowed_shifts = 2'd2; end // Critical, Emergency
        else if (collision_risk == 3'b010) begin max_allowed_shifts = 2'd1; end // Danger
        else begin max_allowed_shifts = 2'd0; end

        if (gear_cooldown_cnt == 0) begin
            if (shift_count < max_allowed_shifts) begin
                if (sensor_data_in.gear > 2'd1) begin can_downshift = 1'b1; end // 1단 초과일 때만 허용
                else begin can_downshift = 1'b0; end
            end else begin can_downshift = 1'b0; end
        end else begin can_downshift = 1'b0; end
    end

    // =========================================================================
    // 4. 각 위험도별 개별 제어 변수 산출
    // =========================================================================
    logic [3:0] Ri__accelerator_collision, Ri__brake_collision;
    logic [1:0] Ri_gear_collision;
    logic       Ri__hazard_collision;

    logic [3:0] Ri__accelerator_road, Ri__brake_road;
    logic [7:0] Ri__speed_limit_road;

    logic [3:0] Ri__accelerator_vision;
    logic [7:0] Ri__speed_limit_vision;
    logic       Ri__headlight_vision, Ri__hazard_vision;

    logic [3:0] Ri__accelerator_posture;
    logic [4:0] Ri__steering_limit_posture;

    // [충돌 위험 제어]
    always_comb begin
        Ri__accelerator_collision = 4'd10; Ri__brake_collision = 4'd0;
        Ri_gear_collision = 2'd0; Ri__hazard_collision = 1'b0;

        case (collision_risk)
            3'b001: begin Ri__accelerator_collision = 4'd0; end
            3'b010: begin 
                Ri__accelerator_collision = 4'd0;
                if (sensor_data_in.speed <= 40) begin Ri__brake_collision = 4'd2; end 
                else if (sensor_data_in.speed <= 80) begin Ri__brake_collision = 4'd3; end 
                else begin Ri__brake_collision = 4'd4; end
                
                if (sensor_data_in.rpm <= 3999) begin
                    if (can_downshift) begin Ri_gear_collision = 2'd1; end 
                end
            end
            3'b011: begin 
                Ri__accelerator_collision = 4'd0;
                if (sensor_data_in.speed <= 40) begin Ri__brake_collision = 4'd4; end 
                else if (sensor_data_in.speed <= 80) begin Ri__brake_collision = 4'd6; end 
                else begin Ri__brake_collision = 4'd8; end
                
                if (sensor_data_in.rpm <= 3999) begin
                    if (can_downshift) begin Ri_gear_collision = 2'd1; end 
                end
                Ri__hazard_collision = 1'b1;
            end
            3'b100: begin 
                Ri__accelerator_collision = 4'd0; Ri__brake_collision = 4'd10;
                
                if (sensor_data_in.rpm <= 3999) begin
                    if (can_downshift) begin Ri_gear_collision = 2'd1; end 
                end
                Ri__hazard_collision = 1'b1;
            end
            default: ; 
        endcase
    end

    // [노면 위험 제어]
    always_comb begin
        logic [3:0] temp_accel_A, temp_accel_B;
        logic [7:0] temp_speed_A, temp_speed_B;
        logic [3:0] temp_brake_B;

        temp_accel_A = 4'd10; temp_speed_A = sensor_data_in.speed_limit;
        temp_accel_B = 4'd10; temp_speed_B = sensor_data_in.speed_limit; temp_brake_B = 4'd0;

        case (road_risk_A)
            2'b01: begin // Wet
                temp_speed_A = ({2'd0, sensor_data_in.speed_limit} * 10'd922) >> 10;
                if (sensor_data_in.speed > temp_speed_A) begin temp_accel_A = 4'd0; end else begin temp_accel_A = 4'd8; end
            end
            2'b10: begin // Ice
                temp_speed_A = ({2'd0, sensor_data_in.speed_limit} * 10'd717) >> 10;
                if (sensor_data_in.speed > temp_speed_A) begin temp_accel_A = 4'd0; end else begin temp_accel_A = 4'd6; end
            end
            2'b11: begin // Black Ice
                temp_speed_A = sensor_data_in.speed_limit >> 1;
                if (sensor_data_in.speed > temp_speed_A) begin temp_accel_A = 4'd0; end else begin temp_accel_A = 4'd4; end
            end
            default: ;
        endcase

        case (road_risk_B)
            2'b01: begin // 거친 노면
                temp_speed_B = ({2'd0, sensor_data_in.speed_limit} * 10'd819) >> 10;
                temp_accel_B = 4'd9;
                if (sensor_data_in.speed > temp_speed_B) begin temp_brake_B = 4'd2; end
            end
            2'b10: begin // 심한 충격
                temp_speed_B = ({2'd0, sensor_data_in.speed_limit} * 10'd614) >> 10;
                temp_accel_B = 4'd7;
                if (sensor_data_in.speed > temp_speed_B) begin temp_brake_B = 4'd2; end
            end
            2'b11: begin // 극심한 충격
                temp_speed_B = sensor_data_in.speed_limit >> 1;
                temp_accel_B = 4'd5;
                if (sensor_data_in.speed > temp_speed_B) begin temp_brake_B = 4'd2; end
            end
            default: ;
        endcase

        if (temp_accel_A < temp_accel_B) begin Ri__accelerator_road = temp_accel_A; end 
        else begin Ri__accelerator_road = temp_accel_B; end

        if (temp_speed_A < temp_speed_B) begin Ri__speed_limit_road = temp_speed_A; end 
        else begin Ri__speed_limit_road = temp_speed_B; end

        Ri__brake_road = temp_brake_B; 
    end

    // [시야 위험 제어]
    always_comb begin
        logic [3:0] temp_accel_A, temp_accel_B;
        logic [7:0] temp_speed_A, temp_speed_B;
        logic temp_head_A, temp_head_B, temp_haz_B;

        temp_accel_A = 4'd10; temp_speed_A = sensor_data_in.speed_limit; temp_head_A = 1'b0;
        temp_accel_B = 4'd10; temp_speed_B = sensor_data_in.speed_limit; temp_head_B = 1'b0; temp_haz_B = 1'b0;

        case (vision_risk_A)
            2'b00: begin temp_head_A = 1'b0; end
            2'b01, 2'b10: begin temp_head_A = 1'b1; end
            2'b11: begin 
                temp_head_A = 1'b1;
                temp_speed_A = ({2'd0, sensor_data_in.speed_limit} * 10'd922) >> 10;
                if (sensor_data_in.speed > temp_speed_A) begin temp_accel_A = 4'd0; end else begin temp_accel_A = 4'd10; end
            end
        endcase

        case (vision_risk_B)
            2'd1: begin // Fog
                temp_head_B = 1'b1; temp_haz_B = 1'b1;
                temp_speed_B = ({2'd0, sensor_data_in.speed_limit} * 10'd614) >> 10;
                if (sensor_data_in.speed > temp_speed_B) begin temp_accel_B = 4'd0; end else begin temp_accel_B = 4'd8; end
            end
            2'd2: begin // Rain
                temp_head_B = 1'b1; temp_haz_B = 1'b0;
                temp_speed_B = ({2'd0, sensor_data_in.speed_limit} * 10'd922) >> 10;
                if (sensor_data_in.speed > temp_speed_B) begin temp_accel_B = 4'd0; end else begin temp_accel_B = 4'd8; end
            end
            2'd3: begin // Snow
                temp_head_B = 1'b0; temp_haz_B = 1'b0;
                temp_speed_B = ({2'd0, sensor_data_in.speed_limit} * 10'd614) >> 10;
                if (sensor_data_in.speed > temp_speed_B) begin temp_accel_B = 4'd0; end else begin temp_accel_B = 4'd5; end
            end
            default: ; 
        endcase

        if (temp_accel_A < temp_accel_B) begin Ri__accelerator_vision = temp_accel_A; end 
        else begin Ri__accelerator_vision = temp_accel_B; end

        if (temp_speed_A < temp_speed_B) begin Ri__speed_limit_vision = temp_speed_A; end 
        else begin Ri__speed_limit_vision = temp_speed_B; end
        
        Ri__headlight_vision = temp_head_A | temp_head_B;
        Ri__hazard_vision = temp_haz_B;
    end

    // [자세 위험 제어]
    always_comb begin
        logic [3:0] temp_accel;
        logic [4:0] temp_steer; 

        temp_accel = 4'd10; temp_steer = 5'd10; 

        if (posture_risk_A == 1'b1) begin 
            if (temp_accel > 4'd0) begin temp_accel = 4'd0; end
            if (temp_steer > 5'd5) begin temp_steer = 5'd5; end
        end

        if (posture_risk_B == 2'b01) begin 
            if (temp_accel > 4'd8) begin temp_accel = 4'd8; end
            if (temp_steer > 5'd7) begin temp_steer = 5'd7; end
        end else if (posture_risk_B == 2'b10) begin 
            if (temp_accel > 4'd0) begin temp_accel = 4'd0; end
            if (temp_steer > 5'd5) begin temp_steer = 5'd5; end
        end

        if (posture_risk_C == 2'b01) begin 
            if (temp_accel > 4'd7) begin temp_accel = 4'd7; end
            if (temp_steer > 5'd8) begin temp_steer = 5'd8; end
        end else if (posture_risk_C == 2'b10) begin 
            if (temp_accel > 4'd0) begin temp_accel = 4'd0; end
            if (temp_steer > 5'd6) begin temp_steer = 5'd6; end
        end

        Ri__accelerator_posture = temp_accel;
        Ri__steering_limit_posture = temp_steer;
    end

    // =========================================================================
    // 5. 최종 제어 중재 (Arbitration Logic)
    // =========================================================================
    logic [3:0] calc_accel_limit;
    logic [3:0] calc_brake_int;
    logic [7:0] calc_speed_limit;
    logic [1:0] calc_gear_down;
    
    logic [3:0] final_accel;
    logic [3:0] final_brake;
    logic [7:0] final_speed_limit;
    logic [1:0] final_gear;
    logic       final_headlight;
    logic       final_hazard;

    always_comb begin
        logic [3:0] min_accel_1, min_accel_2;

        if (Ri__accelerator_collision < Ri__accelerator_road) begin min_accel_1 = Ri__accelerator_collision; end 
        else begin min_accel_1 = Ri__accelerator_road; end

        if (Ri__accelerator_vision < Ri__accelerator_posture) begin min_accel_2 = Ri__accelerator_vision; end 
        else begin min_accel_2 = Ri__accelerator_posture; end

        if (min_accel_1 < min_accel_2) begin calc_accel_limit = min_accel_1; end 
        else begin calc_accel_limit = min_accel_2; end

        if (Ri__speed_limit_road < Ri__speed_limit_vision) begin calc_speed_limit = Ri__speed_limit_road; end 
        else begin calc_speed_limit = Ri__speed_limit_vision; end

        if (Ri__brake_collision > Ri__brake_road) begin calc_brake_int = Ri__brake_collision; end 
        else begin calc_brake_int = Ri__brake_road; end

        calc_gear_down = Ri_gear_collision;

        if (sensor_data_in.accelerator > calc_accel_limit) begin final_accel = calc_accel_limit; end 
        else begin final_accel = sensor_data_in.accelerator; end

        if (sensor_data_in.speed_limit > calc_speed_limit) begin final_speed_limit = calc_speed_limit; end 
        else begin final_speed_limit = sensor_data_in.speed_limit; end

        if (sensor_data_in.brake < calc_brake_int) begin final_brake = calc_brake_int; end 
        else begin final_brake = sensor_data_in.brake; end

        if (calc_gear_down > 2'd0) begin
            if (sensor_data_in.gear > calc_gear_down) begin final_gear = sensor_data_in.gear - calc_gear_down; end 
            else begin final_gear = 2'd1; end
        end else begin
            final_gear = sensor_data_in.gear;
        end

        final_headlight = sensor_data_in.headlight | Ri__headlight_vision;
        final_hazard = sensor_data_in.hazard | Ri__hazard_collision | Ri__hazard_vision;
    end

    // =========================================================================
    // 6. 최종 출력 플립플롭 및 순차 상태 제어
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n || is_trash) begin 
            sensor_data_out.accelerator <= 4'd0;
            sensor_data_out.brake <= 4'd5;
            sensor_data_out.steering <= 5'd15;
            sensor_data_out.speed_limit <= sensor_data_in.speed_limit;
            sensor_data_out.gear <= 2'd0;
            sensor_data_out.headlight <= 1'b1;
            sensor_data_out.hazard <= 1'b0;
            sensor_data_out.manual_mode <= sensor_data_in.manual_mode;
            
            steering_limit_out <= 5'd10; 
            sample_seq_s <= sample_seq_s1;
            
            gear_cooldown_cnt <= 0;
            shift_count <= 2'd0;
            prev_collision_risk <= 3'b000;
        end
        else begin
            sensor_data_out.accelerator <= final_accel;
            sensor_data_out.brake <= final_brake;
            sensor_data_out.speed_limit <= final_speed_limit;
            sensor_data_out.gear <= final_gear;
            sensor_data_out.headlight <= final_headlight;
            sensor_data_out.hazard <= final_hazard;
            
            sensor_data_out.steering <= sensor_data_in.steering;
            sensor_data_out.manual_mode <= sensor_data_in.manual_mode;
            
            steering_limit_out <= Ri__steering_limit_posture;
            sample_seq_s <= sample_seq_s1;
            
            prev_collision_risk <= collision_risk;
            
            // 다중 기어 강하 상태 머신 & 쿨다운 타이머
            if (collision_risk < 3'b010 || collision_risk != prev_collision_risk) begin
                shift_count <= 2'd0;
                
                if (gear_cooldown_cnt > 0) begin
                    gear_cooldown_cnt <= gear_cooldown_cnt - 1;
                end
            end else begin
                if (gear_cooldown_cnt > 0) begin
                    gear_cooldown_cnt <= gear_cooldown_cnt - 1;
                end else if (calc_gear_down > 2'd0 && shift_count < max_allowed_shifts) begin
                    shift_count <= shift_count + 2'd1;
                    gear_cooldown_cnt <= clk_freq / 2;
                end
            end
        end
    end

endmodule