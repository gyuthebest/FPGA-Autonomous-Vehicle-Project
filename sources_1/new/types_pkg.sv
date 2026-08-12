`timescale 1ns / 1ps

package types_pkg;

    localparam HISTORY = 10;
    //============================================================
    // Raw Sensor Data
    //============================================================

    typedef struct packed {

        logic [14:0] distance;

        logic signed [12:0] approach_speed;

        logic signed [11:0] accel_x;
        logic signed [11:0] accel_y;
        logic signed [11:0] accel_z;

        logic signed [15:0] gyro_x;
        logic signed [15:0] gyro_y;
        logic signed [15:0] gyro_z;

        logic signed [10:0] temperature;

        logic [6:0] humidity;

        logic [17:0] lux;

    } sensor_data_t;


    //============================================================
    // Simulation Input
    //============================================================

    typedef struct packed {

        logic signed [13:0] speed_x;
        logic signed [13:0] speed_y;
        logic signed [13:0] speed_z;

        logic signed [15:0] incline_x;
        logic signed [15:0] incline_y;
        logic signed [15:0] incline_z;

        logic [12:0] speed_limit;

        logic [1:0] weather;

        logic [1:0] rpm;

        logic [3:0] accelerator;

        logic [3:0] brake;

        logic signed [7:0] steering;

        logic manual_mode;

        logic [1:0] gear;

        logic headlight;

        logic hazard;

        logic [2:0] situation; //수정 2비트에서 3비트로 늘림

    } sim_data_t;


    //============================================================
    // Preprocessor Output
    //============================================================

    typedef struct packed{

        //----------------------------
        // Derived Values
        //----------------------------

        logic signed [15:0] delta_distance;

        logic signed [13:0] delta_approach_speed;

        logic signed [12:0] delta_accel_x;
        logic signed [12:0] delta_accel_y;
        logic signed [12:0] delta_accel_z;

        logic signed [16:0] delta_gyro_x;
        logic signed [16:0] delta_gyro_y;
        logic signed [16:0] delta_gyro_z;

        logic signed [11:0] delta_temp;
        logic signed [7:0] delta_hum;
        logic signed [18:0] delta_lux;

        logic signed [14:0] delta_speed_x;
        logic signed [14:0] delta_speed_y;
        logic signed [14:0] delta_speed_z;

        logic signed [16:0] delta_incline_x;
        logic signed [16:0] delta_incline_y;
        logic signed [16:0] delta_incline_z;

        logic signed [8:0] delta_steering;   

    } processed_data_t;

    typedef struct packed {
        
        logic signed [15:0] pred_distance;

        logic signed [12:0] pred_approach_speed;

        logic signed [11:0] pred_accel_x_1;
        logic signed [11:0] pred_accel_x_2;
        logic signed [11:0] pred_accel_x_3;
        logic signed [11:0] pred_accel_y_1;
        logic signed [11:0] pred_accel_y_2;
        logic signed [11:0] pred_accel_y_3;
        logic signed [11:0] pred_accel_z_1;
        logic signed [11:0] pred_accel_z_2;

        logic signed [15:0] pred_gyro_x_1;
        logic signed [15:0] pred_gyro_x_2;
        logic signed [15:0] pred_gyro_y_1;
        logic signed [15:0] pred_gyro_y_2;
        logic signed [15:0] pred_gyro_z_1;
        logic signed [15:0] pred_gyro_z_2;
        logic signed [15:0] pred_gyro_z_3;
    
    } pred_data_t;
    //============================================================
    // Reliability Output
    //============================================================

    //typedef struct packed {

        //logic [7:0] ttc_risk; 

        //logic [7:0] braking_risk;

        //logic [7:0] visibility_risk;

        //logic [7:0] posture_risk;

        //logic [7:0] total_risk;

    //} reliability_t;

    typedef struct packed { //추가

        logic [2:0] Ri_collision;

        logic [1:0] Ri_road_A;
        logic [1:0] Ri_road_B;

        logic [1:0] Ri_vision_A;
        logic [1:0] Ri_vision_B;

        logic       Ri_posture_A;
        logic [1:0] Ri_posture_B;
        logic [1:0] Ri_posture_C;

    } risk_t;

    //============================================================
    // Reliability - per channel
    //   각 check 의 confirmed 결과
    //   state : 2'b00 정상, 2'b01 사용제한, 2'b10 사용불가
    //============================================================

    typedef struct packed {

        logic range;         // Range check
        logic jump;          // Jump check
        logic stuck;         // Stuck check
        logic noise;         // Noise check
        logic consistency;   // Consistency check
        logic timeout;       // Timeout check

        logic [1:0] state;      // 종합 판정

    } channel_reliability_t;


    //============================================================
    // Reliability - all channels
    //   필드 순서는 sensor_data_t 와 동일
    //============================================================

    typedef struct packed {

        channel_reliability_t distance;
        channel_reliability_t approach_speed;
        channel_reliability_t accel_x;
        channel_reliability_t accel_y;
        channel_reliability_t accel_z;
        channel_reliability_t gyro_x;
        channel_reliability_t gyro_y;
        channel_reliability_t gyro_z;
        channel_reliability_t temperature;
        channel_reliability_t humidity;
        channel_reliability_t lux;

    } reliability_state_t;

endpackage