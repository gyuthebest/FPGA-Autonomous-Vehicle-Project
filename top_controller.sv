`timescale 1ns / 1ps

import types_pkg::*;

module top_controller #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 5
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
    input  logic                          S_AXI_RREADY,

    //------------------------------------------------------------
    // Pipeline Outputs
    //------------------------------------------------------------
);

    //------------------------------------------------------------
    // AXI Slave → Pipeline 내부 연결 신호
    //------------------------------------------------------------
    sensor_data_t sensor_data_axi;
    logic [31:0]  sample_seq_axi;

    //------------------------------------------------------------
    // AXI Slave 인스턴스
    //------------------------------------------------------------
    sensor_input_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH)
    ) u_axi_slave (
        .sensor_data_out (sensor_data_axi),
        .sample_seq      (sample_seq_axi),

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
    // Preprocessor 인스턴스
    //---------------------------
    sensor_data_t sensor_data_s0;
    processed_data_t process_data_s0;
    logic [31:0] sample_seq_s1;
    
    preprocessor u_preprocessor (
        .clk                (S_AXI_ACLK),
        .rst_n              (S_AXI_ARESETN),
        .sensor_data_in     (sensor_data_axi),
        .sensor_data_out    (sensor_data_s0),
        .processed_data_out (processed_data_s0),
        .sample_seq         (sample_seq_axi),
        .sample_seq_s1      (sample_seq_s1)
    );
    
endmodule