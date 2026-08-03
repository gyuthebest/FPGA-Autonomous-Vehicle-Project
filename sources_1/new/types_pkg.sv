`timescale 1ns / 1ps

package types_pkg;

    //============================================================
    // Raw Sensor Data
    //============================================================
    
    typedef struct packed {

        // Sensor Inputs
        logic [14:0] distance;

        logic [7:0]  speed_x;
        logic [7:0]  speed_y;
        logic [7:0]  speed_z;

        logic signed [15:0] accel_x;
        logic signed [15:0] accel_y;
        logic signed [15:0] accel_z;

        logic signed [15:0] gyro_x;
        logic signed [15:0] gyro_y;
        logic signed [15:0] gyro_z;

        logic signed [15:0] incline_x;
        logic signed [15:0] incline_y;
        logic signed [15:0] incline_z;

        logic signed [10:0] temperature;

        logic [6:0] humidity;

        logic [17:0] lux;

        logic signed [9:0] approach_speed;

        logic [7:0] speed_limit;

        // Simulation Inputs
        logic [1:0] weather;

        logic [1:0] rpm;

        logic [3:0] accelerator;

        logic [3:0] brake;

        logic [4:0] steering;

        logic manual_mode;

        logic [1:0] gear;

        logic headlight;

        logic hazard;

    } sensor_data_t;


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

endpackage