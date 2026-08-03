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

    logic first_sample;

    //------------------------------------------------------------
    // Sequential Logic
    //------------------------------------------------------------

    always_ff @(posedge clk) begin

        if(!rst_n) begin
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
                    valid_s1 <= 1'b0;
                    first_sample <= 1'b0;
                end

                //--------------------------------------------
                // Normal Operation
                //--------------------------------------------

                else begin
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
