`timescale 1ns / 1ps

package types_pkg;

    //============================================================
    // Raw Sensor Data
    //============================================================

    typedef struct packed {

        logic [14:0] distance;

        logic signed [9:0] approach_speed;


        logic [7:0]  speed_x;
        logic [7:0]  speed_y;
        logic [7:0]  speed_z;

        logic signed [15:0] accel_x;
        logic signed [15:0] accel_y;
        logic signed [15:0] accel_z;

        logic signed [15:0] incline_x;
        logic signed [15:0] incline_y;
        logic signed [15:0] incline_z;

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

        logic [7:0] speed_limit;

        logic [1:0] weather;

        logic [1:0] rpm;

        logic [3:0] accelerator;

        logic [3:0] brake;

        logic [4:0] steering;

        logic manual_mode;

        logic [1:0] gear;

        logic headlight;

        logic hazard;

    } sim_data_t;


    //============================================================
    // Preprocessor Output
    //============================================================

    typedef struct packed {

        //----------------------------
        // Derived Values
        //----------------------------

        logic signed [15:0] delta_distance;

        logic signed [8:0]  delta_speed_x;
        logic signed [8:0]  delta_speed_y;
        logic signed [8:0]  delta_speed_z;

        logic signed [16:0] delta_accel_x;
        logic signed [16:0] delta_accel_y;
        logic signed [16:0] delta_accel_z;

        logic signed [16:0] delta_gyro_x;
        logic signed [16:0] delta_gyro_y;
        logic signed [16:0] delta_gyro_z;

        logic signed [16:0] delta_incline_x;
        logic signed [16:0] delta_incline_y;
        logic signed [16:0] delta_incline_z;

        logic signed [11:0] delta_temp;

        logic signed [7:0] delta_hum;

        logic signed [18:0] delta_lux;

        logic signed [10:0] delta_approach_speed;

        logic weather_change;

    } processed_data_t;

    //============================================================
    // Reliability Output
    //============================================================

    typedef struct packed {

        logic [7:0] ttc_risk;

        logic [7:0] braking_risk;

        logic [7:0] visibility_risk;

        logic [7:0] posture_risk;

        logic [7:0] total_risk;

    } reliability_t;

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

        logic [1:0] state;      // 종합 판정

    } channel_reliability_t;


    //============================================================
    // Reliability - all channels
    //   필드 순서는 sensor_data_t 와 동일
    //============================================================

    typedef struct packed {

        channel_reliability_t distance;
        channel_reliability_t approach_speed;
        channel_reliability_t speed_x;
        channel_reliability_t speed_y;
        channel_reliability_t speed_z;
        channel_reliability_t accel_x;
        channel_reliability_t accel_y;
        channel_reliability_t accel_z;
        channel_reliability_t incline_x;
        channel_reliability_t incline_y;
        channel_reliability_t incline_z;
        channel_reliability_t gyro_x;
        channel_reliability_t gyro_y;
        channel_reliability_t gyro_z;
        channel_reliability_t temperature;
        channel_reliability_t humidity;
        channel_reliability_t lux;

        logic is_timeout;       // 시스템 공통

    } reliability_state_t;

endpackage