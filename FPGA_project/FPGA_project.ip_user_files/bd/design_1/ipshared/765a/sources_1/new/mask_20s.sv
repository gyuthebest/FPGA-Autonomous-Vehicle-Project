`timescale 1ns / 1ps

import types_pkg::*;

module mask_20s #(
/* Legacy declaration retained in this comment to avoid encoding damage:
    parameter CLK_FREQ = 88888000, //legacy example: 88.888MHz
)(

*/
    parameter int unsigned CLK_FREQ = 88888000
) (
input logic clk,
input logic rst_n,

input logic stuck_err,
input logic consistency_error_approach_speed,

output logic stuck_mask_20s,
output logic consistency_mask_20s_approach_speed

);

logic [31:0] stuck_mask_20s_cnt;
logic [31:0] consistency_mask_20s_approach_speed_cnt;

assign stuck_mask_20s = (stuck_mask_20s_cnt != 0);
assign consistency_mask_20s_approach_speed = (consistency_mask_20s_approach_speed_cnt != 0);


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
