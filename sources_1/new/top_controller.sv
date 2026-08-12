`timescale 1ns / 1ps

import types_pkg::*;

module top_controller #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
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
    logic         valid_risk;
    logic         valid_rel;

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
        .valid_risk      (valid_risk),
        .valid_rel       (valid_rel),

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
    sensor_data_t    sensor_data_s0;
    sim_data_t       sim_data_s0;
    processed_data_t process_data_s0;
    logic [31:0]     sample_seq_s1;
    logic            valid_s1;
    logic [31:0]     clk_cnt_s1;
    
    preprocessor u_preprocessor (
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),
        
        .sensor_data_in     (sensor_data_axi),
        .sim_data_in        (sim_data_axi),
        .sample_seq         (sample_seq_axi),
        
        .sensor_data_out    (sensor_data_s0),
        .sim_data_out       (sim_data_s0),
        .processed_data_out (process_data_s0),
        .sample_seq_s1      (sample_seq_s1),
        .valid_s1           (valid_s1),
        .clk_cnt_s1         (clk_cnt_s1)
    );

    //---------------------------
    // Sensor Reliability 인스턴스화
    //---------------------------
    reliability_state_t rel_out_s1;
    logic [31:0]        sample_seq_out_rel;
    logic               valid_out_rel;

    sensor_reliability u_sensor_reliability (
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),
        
        .sensor_data_in     (sensor_data_s0),
        .processed_data_in  (process_data_s0),
        .valid_s1           (valid_s1),
        .sample_seq_in      (sample_seq_s1),
        
        .rel_out            (rel_out_s1),
        .sample_seq_out     (sample_seq_out_rel),
        .valid_out          (valid_out_rel)
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
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),
        
        .sim_data_in        (sim_data_s0),
        .sensor_data_in     (sensor_data_s0),
        .processed_data_in  (process_data_s0),
        .valid_in           (valid_s1),
        .sample_seq_in      (sample_seq_s1),
        
        .risk_out           (risk_out_s1),
        .sim_data_out       (sim_data_risk_s1),
        .sensor_data_out    (sensor_data_risk_s1),
        .sample_seq_out     (sample_seq_out_risk_types),
        .valid_out          (valid_out_risk_types)
    );

    //---------------------------
    // Risk Control 2 인스턴스화
    //---------------------------
    risk_control u_risk_control (
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),
        
        .risk_in            (risk_out_s1),
        .rel_in             (rel_out_s1),
        .sim_data_in        (sim_data_risk_s1),
        .sensor_data_in     (sensor_data_risk_s1),
        .sample_seq_in      (sample_seq_out_risk_types),
        .valid_in           (valid_out_risk_types),
        .valid_in_rel       (valid_out_rel),
        .sample_seq_in_rel  (sample_seq_out_rel),
        
        .sensor_data_out    (sensor_data_out),
        .sim_data_out       (sim_data_out),
        .risk_out           (risk_out),
        .rel_out            (rel_out),
        .sample_seq_out     (sample_seq_risk),
        .sample_seq_out_rel (sample_seq_rel),
        .valid_out          (valid_risk),
        .valid_out_rel      (valid_rel)
    );
    
endmodule