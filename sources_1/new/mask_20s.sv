`timescale 1ns / 1ps

import types_pkg::*;

module mask_20s #(
    parameter CLK_FREQ = 100000000, //100MHz 가정
)(

input clk,
input rst_n,

input logic stuck_err,
input logic consistency_error_approach_speed,

output logic stuck_mask_20s,
output logic consistency_mask_20s_approach_speed

);

assign stuck_mask_20s = (stuck_mask_20s_cnt != 0);
assign consistency_mask_20s_approach_speed = (consistency_mask_20s_approach_speed_cnt != 0);

logic [31:0] stuck_mask_20s_cnt;
logic [31:0] consistency_mask_20s_approach_speed_cnt;


always_ff @(posedge clk) begin
    if (!rst_n) 
        stuck_mask_20s_cnt <= '0;
    else if(stuck_err) 
        stuck_mask_20s_cnt <= 20 * CLK_FREQ;
    else if (stuck_mask_20s_cnt > 0)
        stuck_mask_20s_cnt <= stuck_mask_20s_cnt - 1;
        
end

always_ff @(posedge clk) begin
    if(!rst_n)
        consistency_mask_20s_approach_speed_cnt <= '0;
    else if(consistency_error_approach_speed)
        consistency_mask_20s_approach_speed_cnt <= 20 * CLK_FREQ;
    else if(consistency_mask_20s_approach_speed_cnt > 0)
        consistency_mask_20s_approach_speed_cnt <= consistency_mask_20s_approach_speed_cnt -1;
end

endmodule