`timescale 1ns / 1ps

import types_pkg::*;

module consistency_check #(
    parameter WIDTH = 12,
    parameter S = 40,
    parameter CONSISTENCY_THRESHOLD = 360,
    parameter int unsigned CONSISTENCY_U = 1,
    parameter int unsigned CONSISTENCY_D = 3,
    parameter int unsigned CONSISTENCY_N = 8
)(
    input clk,
    input rst_n,
    input valid_s1,
    input timeout_mask_2s,
    input consistency_mask,
    input signed [WIDTH-1:0] sensor_data,
    input signed [WIDTH-1:0] pred_data,
    output logic consistency_error
    
);

    logic [1:0] raw_consistency;
    logic [31:0] consistency_cnt;

    // consistency check
    assign consistency_error = (consistency_cnt >= CONSISTENCY_N);

    function automatic logic signed [31:0] abs(input logic signed [31:0] val); return (val < 0) ? -val : val; endfunction

    always_comb begin
        raw_consistency = 2'b00;  
        if (valid_s1) begin
            if (timeout_mask_2s) raw_consistency = 2'b10;
            else begin
                if (abs(sensor_data * S - pred_data) <= CONSISTENCY_THRESHOLD) raw_consistency = 2'b00;
                else if ((abs(sensor_data * S - pred_data) > CONSISTENCY_THRESHOLD) && !consistency_mask) raw_consistency = 2'b01;
                else raw_consistency = 2'b10;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            consistency_cnt <= '0;
        end
        else begin
            if (valid_s1) begin
                if (raw_consistency == 2'b00) consistency_cnt <= (consistency_cnt < CONSISTENCY_D) ? '0 : consistency_cnt - CONSISTENCY_D;
                else if (raw_consistency == 2'b01) consistency_cnt <= consistency_cnt + CONSISTENCY_U;
                else consistency_cnt <= consistency_cnt;
            end
        end
    end
endmodule