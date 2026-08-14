`timescale 1ns / 1ps

import types_pkg::*;

module risk_control #(
    parameter int CLK_FREQ = 100000000
) (
    input logic clk,
    input logic rst_n,

    input logic valid_in,
    input logic valid_in_rel,
    input logic [31:0] sample_seq_in,
    input logic [31:0] sample_seq_in_rel,
    input sensor_data_t sensor_data_in,
    input sim_data_t sim_data_in,

    input reliability_state_t rel_in,
    input risk_t risk_in,

    output logic [31:0] sample_seq_out_risk,
    output logic [31:0] sample_seq_out_rel,
    output logic [1:0] valid_out_rel_risk,
    
    output risk_t risk_out,
    output reliability_state_t rel_out,
    output sensor_data_t sensor_data_out,
    output sim_data_t sim_data_out,

    output logic transition_demand,
    output logic hud_warning,
    output logic mrm,
    output logic [3:0] td_remain_sec
);

    risk_control_2 #(
        .CLK_FREQ(CLK_FREQ)
    ) u_real_risk_control (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .valid_in_rel(valid_in_rel),
        .sample_seq_in(sample_seq_in),
        .sample_seq_in_rel(sample_seq_in_rel),
        .sensor_data_in(sensor_data_in),
        .sim_data_in(sim_data_in),
        .rel_in(rel_in),
        .risk_in(risk_in),
        .sample_seq_out_risk(sample_seq_out_risk),
        .sample_seq_out_rel(sample_seq_out_rel),
        .valid_out_rel_risk(valid_out_rel_risk),
        .risk_out(risk_out),
        .rel_out(rel_out),
        .sensor_data_out(sensor_data_out),
        .sim_data_out(sim_data_out),
        .transition_demand(transition_demand),
        .hud_warning(hud_warning),
        .mrm(mrm),
        .td_remain_sec(td_remain_sec)
    );

endmodule
