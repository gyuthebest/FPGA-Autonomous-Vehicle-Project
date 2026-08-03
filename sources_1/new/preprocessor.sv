`timescale 1ns / 1ps

import types_pkg::*;

module preprocessor(

    input  logic clk,
    input  logic rst_n,

    //------------------------------------------------------------
    // Inputs
    //------------------------------------------------------------

    input  sensor_data_t   sensor_data_in,
    input  logic [31:0]    sample_seq,

    //------------------------------------------------------------
    // Outputs
    //------------------------------------------------------------

    output sensor_data_t    sensor_data_out,
    output processed_data_t processed_data_out,
    output logic            valid_s1,
    output logic [31:0]     sample_seq_s1 
);

    //------------------------------------------------------------
    // Calculate valid_s0
    //------------------------------------------------------------
    
    logic [31:0] sample_seq_prev;
    logic valid_s0;

    always_comb begin
        if (sample_seq != sample_seq_s1) begin
            valid_s0 = 1'b1;
        end
        else begin
            valid_s0 = 1'b0;
        end
    end


    //------------------------------------------------------------
    // Internal Registers
    //------------------------------------------------------------

    processed_data_t processed_data;

    logic first_sample;

    //------------------------------------------------------------
    // Next-State Logic
    //------------------------------------------------------------

    
    always_comb begin
        processed_data = processed_data_out;
        
        // Ignore first sample (Can't calculate delta)
        if(valid_s0 && !first_sample) begin

            //--------------------------------------------------------
            // Distance Delta
            //--------------------------------------------------------
            
            processed_data.delta_distance =
                $signed({1'b0, sensor_data_in.distance}) -
                $signed({1'b0, sensor_data_out.distance});
                
            //--------------------------------------------------------
            // Speed Delta
            //--------------------------------------------------------

            processed_data.delta_speed_x =
                $signed({1'b0, sensor_data_in.speed_x}) -
                $signed({1'b0, sensor_data_out.speed_x});

            processed_data.delta_speed_y =
                $signed({1'b0, sensor_data_in.speed_y}) -
                $signed({1'b0, sensor_data_out.speed_y});

            processed_data.delta_speed_z =
                $signed({1'b0, sensor_data_in.speed_z}) -
                $signed({1'b0, sensor_data_out.speed_z});


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
            // Incline Delta
            //--------------------------------------------------------

            processed_data.delta_incline_x =
                sensor_data_in.incline_x -
                sensor_data_out.incline_x;

            processed_data.delta_incline_y =
                sensor_data_in.incline_y -
                sensor_data_out.incline_y;

            processed_data.delta_incline_z =
                sensor_data_in.incline_z -
                sensor_data_out.incline_z;

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
            // Approach Speed Delta
            //--------------------------------------------------------

            processed_data.delta_approach_speed =
                sensor_data_in.approach_speed -
                sensor_data_out.approach_speed;
        end
    end
        
    //------------------------------------------------------------
    // Sequential Logic
    //------------------------------------------------------------

    always_ff @(posedge clk) begin

        if(!rst_n) begin
            processed_data_out  <= '0;
            sensor_data_out     <= '0;
            first_sample        <= 1'b1;
            valid_s1            <= 1'b0;
            sample_seq_s1       <= 32'd0;
        end
        else begin
            
            //----------------------------------------------------
            // Update Only When New Sample Arrives
            //----------------------------------------------------

            if(valid_s0) begin

                // Always
                sample_seq_s1 <= sample_seq;
                sensor_data_out <= sensor_data_in;
                
                //--------------------------------------------
                // First Valid Sample
                //--------------------------------------------

                if(first_sample) begin
                    processed_data_out <= '0;
                    valid_s1 <= 1'b0;
                    first_sample <= 1'b0;
                end

                //--------------------------------------------
                // Normal Operation
                //--------------------------------------------

                else begin
                    processed_data_out <= processed_data;
                    valid_s1 <= valid_s0;
                end
            end

            //----------------------------------------------------
            // No New Sample
            //----------------------------------------------------

            else begin
                valid_s1 <= 1'b0;
            end
        end
    end

endmodule