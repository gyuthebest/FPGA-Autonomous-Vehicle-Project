import types_pkg::*;

module risk_types #(
    parameter clk_freq = 100000000 // 100MHz
)(
    input logic clk,
    input logic rst_n,         
    input logic [31:0] sample_seq_in,  
    input logic valid_in,

    input sensor_data_t sensor_data_in,
    input sim_data_t sim_data_in,
    input processed_data_t processed_data_in,
    
    output sensor_data_t sensor_data_out,
    output sim_data_t sim_data_out,
    output logic [31:0] sample_seq_out,
    output logic valid_out,
    output risk_t risk_out

);

    localparam logic signed [12:0] TTC_4_0S = 13'd4096;
    localparam logic signed [12:0] TTC_3_0S = 13'd3072;
    localparam logic signed [12:0] TTC_2_0S = 13'd2048;
    localparam logic signed [12:0] TTC_1_4S = 13'd1434;

    localparam logic [11:0] ACCEL_0_5G = 12'd490;
    localparam logic [11:0] ACCEL_0_8G = 12'd784;
    localparam logic [11:0] ACCEL_1_0G = 12'd980;
    localparam logic [11:0] ACCEL_2_0G = 12'd1960;

    
    localparam logic [15:0] GYRO_30DEGS = 16'd524;
    localparam logic [15:0] GYRO_40DEGS = 16'd698;
    localparam logic [15:0] GYRO_60DEGS = 16'd1047;


    logic [2:0] collision_risk;
    logic [1:0] road_risk_A;
    logic [1:0] road_risk_B;
    logic [1:0] vision_risk_A;
    logic [1:0] vision_risk_B;
    logic posture_risk_A;
    logic [1:0] posture_risk_B;
    logic [1:0] posture_risk_C;

    
    logic [14:0] abs_gyro_x, abs_gyro_z;
    logic [10:0] abs_accel_y;

    logic signed [12:0] net_accel_z; // 중력을 뺀 accel_z
    logic [11:0] abs_net_accel_z;

  
    // making abs, net
    always_comb begin
        if (sensor_data_in.gyro_x[15] == 1'b1)
            abs_gyro_x = ~(sensor_data_in.gyro_x) + 16'd1; 
        else
            abs_gyro_x = sensor_data_in.gyro_x;
        
        if (sensor_data_in.gyro_z[15] == 1'b1)
            abs_gyro_z = ~(sensor_data_in.gyro_z) + 16'd1;
        else
            abs_gyro_z = sensor_data_in.gyro_z;
        
        if (sensor_data_in.accel_y[11] == 1'b1)
            abs_accel_y = ~(sensor_data_in.accel_y) + 12'd1; 
        else
            abs_accel_y = sensor_data_in.accel_y;


        net_accel_z = sensor_data_in.accel_z - 13'sd980;

        if (net_accel_z[12] == 1'b1)
            abs_net_accel_z = ~(net_accel_z[11:0]) + 12'd1;
        else
            abs_net_accel_z = net_accel_z[11:0];
    end


    // risk types
    always_comb begin

        // collision risk
        if (sensor_data_in.app_speed <= 0)
            collision_risk = 3'b000;
        else begin
            if (sensor_data_in.distance <= (26'(TTC_1_4S) * sensor_data_in.app_speed) >> 10)
                collision_risk = 3'b100; // Emergency
            else if (sensor_data_in.distance <= (26'(TTC_2_0S) * sensor_data_in.app_speed) >> 10)
                collision_risk = 3'b011; // Critical
            else if (sensor_data_in.distance <= (26'(TTC_3_0S) * sensor_data_in.app_speed) >> 10)
                collision_risk = 3'b010; // Danger
            else if (sensor_data_in.distance <= (26'(TTC_4_0S) * sensor_data_in.app_speed) >> 10)
                collision_risk = 3'b001; // Caution
            else
                collision_risk = 3'b000; // Safe
        end

        // road risk
        if (sensor_data_in.temperature <= -50 && sensor_data_in.humidity >= 90)
            road_risk_A = 2'b11; // Black Ice
        else if (sensor_data_in.temperature <= 0 && sensor_data_in.humidity >= 70)
            road_risk_A = 2'b10; // Ice
        else if (sensor_data_in.humidity >= 70)
            road_risk_A = 2'b01;// Wet
        else
            road_risk_A = 2'b00; // Dry


        if (sensor_data_in.speed_x < 833)
            road_risk_B = 2'b00; 
        else begin 
            if (abs_net_accel_z >= ACCEL_2_0G)
                road_risk_B = 2'b11; // 극심한 충격
            else if (abs_net_accel_z >= ACCEL_1_0G)
                road_risk_B = 2'b10; // 심한 충격
            else if (abs_net_accel_z >= ACCEL_0_5G)
                road_risk_B = 2'b01; // 거친 노면
            else 
                road_risk_B = 2'b00;
        end

        // vision risk
        if (sensor_data_in.lux >= 20000)
            vision_risk_A = 2'b00;
        else if (sensor_data_in.lux >= 1000)
            vision_risk_A = 2'b01;
        else if (sensor_data_in.lux >= 50)
            vision_risk_A = 2'b10;
        else
            vision_risk_A = 2'b11;

        vision_risk_B = sim_data_in.weather;


        // posture risk
        if (abs_gyro_x >= GYRO_40DEGS)
            posture_risk_A = 1'b1; // 위험
        else
            posture_risk_A = 1'b0; // 안전

        if (abs_gyro_z >= GYRO_60DEGS)
            posture_risk_B = 2'b10; // 위험
        else if (abs_gyro_z >= GYRO_30DEGS)
            posture_risk_B = 2'b01; // 주의
        else
            posture_risk_B = 2'b00; // 정상

        if (abs_accel_y >= ACCEL_0_8G)
            posture_risk_C = 2'b10; // 위험
        else if (abs_accel_y >= ACCEL_0_5G)
            posture_risk_C = 2'b01; // 주의
        else
            posture_risk_C = 2'b00; // 정상
    end

   
    
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

            sensor_data_out <= sensor_data_in;

            sample_seq_out <= '0;
            valid_out <= 1'b0;
            
            risk_out.Ri_collision <= '0;
            risk_out.Ri_road_A <= '0;
            risk_out.Ri_road_B <= '0;
            risk_out.Ri_vision_A <= '0;
            risk_out.Ri_vision_B <= '0;
            risk_out.Ri_posture_A <= '0;
            risk_out.Ri_posture_B <= '0;
            risk_out.Ri_posture_C <= '0;
   
        end
        else begin
            sim_data_out <= sim_data_in;

            sensor_data_out <= sensor_data_in;
           
            sample_seq_out <= sample_seq_in;
            valid_out <= valid_in;

            risk_out.Ri_collision <= collision_risk;
            risk_out.Ri_road_A <= road_risk_A;
            risk_out.Ri_road_B <= road_risk_B;
            risk_out.Ri_vision_A <= vision_risk_A;
            risk_out.Ri_vision_B <= vision_risk_B;
            risk_out.Ri_posture_A <= posture_risk_A;
            risk_out.Ri_posture_B <= posture_risk_B;
            risk_out.Ri_posture_C <= posture_risk_C;
            
        end
    end

endmodule