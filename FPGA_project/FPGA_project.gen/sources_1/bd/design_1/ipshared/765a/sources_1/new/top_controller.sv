`timescale 1ns / 1ps

import types_pkg::*;

module top_controller #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter integer CLK_FREQ_HZ        = 88888000,
    parameter integer SAMPLE_RATE_HZ     = 20
)(
    //------------------------------------------------------------
    // AXI Clock / Reset
    //------------------------------------------------------------
    input logic S_AXI_ACLK,
    input logic S_AXI_ARESETN,

    //------------------------------------------------------------
    // AXI Write Address Channel
    //------------------------------------------------------------
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  logic [2:0]                    S_AXI_AWPROT,
    input  logic                          S_AXI_AWVALID,
    output logic                          S_AXI_AWREADY,

    //------------------------------------------------------------
    // AXI Write Data Channel
    //------------------------------------------------------------
    input  logic [C_S_AXI_DATA_WIDTH-1:0]   S_AXI_WDATA,
    input  logic [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  logic                            S_AXI_WVALID,
    output logic                            S_AXI_WREADY,

    //------------------------------------------------------------
    // AXI Write Response Channel
    //------------------------------------------------------------
    output logic [1:0] S_AXI_BRESP,
    output logic       S_AXI_BVALID,
    input  logic       S_AXI_BREADY,

    //------------------------------------------------------------
    // AXI Read Address Channel
    //------------------------------------------------------------
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  logic [2:0]                    S_AXI_ARPROT,
    input  logic                          S_AXI_ARVALID,
    output logic                          S_AXI_ARREADY,

    //------------------------------------------------------------
    // AXI Read Data Channel
    //------------------------------------------------------------
    output logic [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output logic [1:0]                    S_AXI_RRESP,
    output logic                          S_AXI_RVALID,
    input  logic                          S_AXI_RREADY

    //------------------------------------------------------------
    // Pipeline Outputs
    //------------------------------------------------------------
);

    //------------------------------------------------------------
    // AXI Slave 인스턴스화
    //------------------------------------------------------------
    sensor_data_t sensor_data_axi;
    sim_data_t    sim_data_axi;
    logic [31:0]  sample_seq_axi;
    
    // Outputs from risk_control
    sensor_data_t sensor_data_out;
    sim_data_t    sim_data_out;
    risk_t        risk_out;
    reliability_state_t rel_out;
    logic [31:0]  sample_seq_risk;
    logic [31:0]  sample_seq_rel;
    logic [1:0]   valid_out_rel_risk;
    logic transition_demand;
    logic hud_warning;
    logic mrm;
    logic [3:0] td_remain_sec;

    sensor_input_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH)
    ) u_axi_slave (
        // Outputs to PL
        .sensor_data_out (sensor_data_axi),
        .sim_data_out    (sim_data_axi),
        .sample_seq      (sample_seq_axi),
        
        // Inputs from PL
        .risk_in         (risk_out),
        .rel_in          (rel_out),
        .sensor_data_in  (sensor_data_out),
        .sim_data_in     (sim_data_out),
        .sample_seq_risk (sample_seq_risk),
        .sample_seq_rel  (sample_seq_rel),
        .valid_out_rel_risk (valid_out_rel_risk),
        .transition_demand (transition_demand),
        .hud_warning      (hud_warning),
        .mrm              (mrm),
        .td_remain_sec    (td_remain_sec),

        // AXI interface
        .S_AXI_ACLK      (S_AXI_ACLK),
        .S_AXI_ARESETN   (S_AXI_ARESETN),

        .S_AXI_AWADDR    (S_AXI_AWADDR),
        .S_AXI_AWPROT    (S_AXI_AWPROT),
        .S_AXI_AWVALID   (S_AXI_AWVALID),
        .S_AXI_AWREADY   (S_AXI_AWREADY),

        .S_AXI_WDATA     (S_AXI_WDATA),
        .S_AXI_WSTRB     (S_AXI_WSTRB),
        .S_AXI_WVALID    (S_AXI_WVALID),
        .S_AXI_WREADY    (S_AXI_WREADY),

        .S_AXI_BRESP     (S_AXI_BRESP),
        .S_AXI_BVALID    (S_AXI_BVALID),
        .S_AXI_BREADY    (S_AXI_BREADY),

        .S_AXI_ARADDR    (S_AXI_ARADDR),
        .S_AXI_ARPROT    (S_AXI_ARPROT),
        .S_AXI_ARVALID   (S_AXI_ARVALID),
        .S_AXI_ARREADY   (S_AXI_ARREADY),

        .S_AXI_RDATA     (S_AXI_RDATA),
        .S_AXI_RRESP     (S_AXI_RRESP),
        .S_AXI_RVALID    (S_AXI_RVALID),
        .S_AXI_RREADY    (S_AXI_RREADY)
    );
    
    //---------------------------
    // Preprocessor 인스턴스화
    //---------------------------
    sensor_data_t    sensor_data_s1;
    sim_data_t       sim_data_s1;
    processed_data_t process_data_s1;
    logic [31:0]     sample_seq_s1;
    logic            valid_s1;
    logic [31:0]     clk_cnt_s1;
    pred_data_t      pred_data_s1;
    logic            timeout_mask_1s;
    logic            consistency_mask_1s_approach_speed;
    logic            consistency_mask_20s_approach_speed;
    processed_data_t prev_processed_data_out;
    logic            consistency_mask_1;
    logic            consistency_mask_2;
    logic            consistency_mask_3;


    
    preprocessor u_preprocessor (
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),

        .timeout_mask_1s (timeout_mask_1s),
        .consistency_mask_1s_approach_speed (consistency_mask_1s_approach_speed), 
        .consistency_mask_20s_approach_speed (consistency_mask_20s_approach_speed), 
        
        .sensor_data_in     (sensor_data_axi),
        .sim_data_in        (sim_data_axi),
        .sample_seq_in         (sample_seq_axi),
        
        .sensor_data_out    (sensor_data_s1),
        .sim_data_out       (sim_data_s1),
        .processed_data_out (process_data_s1),
        .sample_seq_out      (sample_seq_s1),
        .valid_s1           (valid_s1),
        .clk_cnt         (clk_cnt_s1),

        .pred_data_out (pred_data_s1),
        .prev_processed_data_out (prev_processed_data_out),
        .consistency_mask_1 (consistency_mask_1),
        .consistency_mask_2 (consistency_mask_2),
        .consistency_mask_3 (consistency_mask_3)
    );

    //---------------------------
    // Sensor Reliability 인스턴스화
    //---------------------------
    reliability_state_t rel_out_s1;
    logic [31:0]        sample_seq_out_rel;
    logic               valid_out_rel;
    reliability_state_t               reliability_out;
    

    sensor_reliability #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .SAMPLE_RATE_HZ (SAMPLE_RATE_HZ)
    ) u_sensor_reliability (
        .clk                (S_AXI_ACLK),//
        .rst_n              (S_AXI_ARESETN),//
        .clk_cnt (clk_cnt_s1), //수정
        
        .sensor_data_in     (sensor_data_s1),//
        .processed_data_in  (process_data_s1),//
        .prev_processed_data_in (prev_processed_data_out), //
        .valid_s1           (valid_s1),//
        .sample_seq_in      (sample_seq_s1),//
        .pred_data_in       (pred_data_s1), // 

        .consistency_mask_1 (consistency_mask_1),
        .consistency_mask_2 (consistency_mask_2),
        .consistency_mask_3 (consistency_mask_3),
        .situation (sim_data_s1.situation),
        .timeout_mask_1s (timeout_mask_1s), 
        .consistency_mask_1s_approach_speed (consistency_mask_1s_approach_speed), 
        .consistency_mask_20s_approach_speed (consistency_mask_20s_approach_speed),

        
        
        .sample_seq_out_rel     (sample_seq_out_rel),//
        .valid_out_rel          (valid_out_rel),//
        .reliability_out        (reliability_out)//
    );

    //---------------------------
    // Risk Types 인스턴스화
    //---------------------------
    risk_t          risk_out_s1;
    sensor_data_t   sensor_data_risk_s1;
    sim_data_t      sim_data_risk_s1;
    logic [31:0]    sample_seq_out_risk_types;
    logic           valid_out_risk_types;


    

    risk_types u_risk_types (
        .clk                (S_AXI_ACLK),//
        .rst_n              (S_AXI_ARESETN),//
        
        .sim_data_in        (sim_data_s1),//
        .sensor_data_in     (sensor_data_s1),//
        
        .valid_in           (valid_s1),//
        .sample_seq_in      (sample_seq_s1),//
        
        .risk_out           (risk_out_s1),//
        .sim_data_out       (sim_data_risk_s1),//
        .sensor_data_out    (sensor_data_risk_s1),//
        .sample_seq_out     (sample_seq_out_risk_types),//
        .valid_out          (valid_out_risk_types)//
    );

    //---------------------------
    // Risk Control 2 인스턴스화
    //---------------------------

    risk_control_2 #(
        .CLK_FREQ (CLK_FREQ_HZ)
    ) u_risk_control (
        .clk                (S_AXI_ACLK),//
        .rst_n              (S_AXI_ARESETN),//
        
        .risk_in            (risk_out_s1),//
        .rel_in             (reliability_out),//
        .sim_data_in        (sim_data_risk_s1),//
        .sensor_data_in     (sensor_data_risk_s1),//
        .sample_seq_in      (sample_seq_out_risk_types),//
        .valid_in           (valid_out_risk_types),//
        .valid_in_rel       (valid_out_rel),//
        .sample_seq_in_rel  (sample_seq_out_rel),//
        
        .sensor_data_out    (sensor_data_out),//
        .sim_data_out       (sim_data_out),//
        .risk_out           (risk_out),//
        .rel_out            (rel_out),//
        .sample_seq_out_risk     (sample_seq_risk), //
        .sample_seq_out_rel (sample_seq_rel),//
        .valid_out_rel_risk (valid_out_rel_risk), //

        .transition_demand (transition_demand),
        .hud_warning        (hud_warning),
        .mrm                (mrm),
        .td_remain_sec      (td_remain_sec)
    );
    
endmodule
