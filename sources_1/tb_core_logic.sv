`timescale 1ns / 1ps

import types_pkg::*;

module tb_core_logic;

    //------------------------------------------------------------
    // 1. Clock & Reset 
    //------------------------------------------------------------
    logic clk;
    logic rst_n;
    
    // 100MHz Clock Generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    //------------------------------------------------------------
    // 2. TB <-> Preprocessor Inputs
    //------------------------------------------------------------
    sensor_data_t sensor_data_axi;
    sim_data_t    sim_data_axi;
    logic [31:0]  sample_seq_axi;
    
    // Outputs from risk_control_2
    sensor_data_t sensor_data_out;
    sim_data_t    sim_data_out;
    risk_t        risk_out;
    reliability_state_t rel_out;
    logic [31:0]  sample_seq_risk;
    logic [31:0]  sample_seq_rel;
    logic [1:0]   valid_rel_risk;
    
    // Interconnect wires
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
    pred_data_t      prev_processed_data_out;
    logic            consistency_mask_1;
    logic            consistency_mask_2;
    logic            consistency_mask_3;

    reliability_state_t reliability_out;
    logic [31:0]        sample_seq_out_rel;
    logic               valid_out_rel;

    risk_t          risk_out_s1;
    sensor_data_t   sensor_data_risk_s1;
    sim_data_t      sim_data_risk_s1;
    logic [31:0]    sample_seq_out_risk_types;
    logic           valid_out_risk_types;
    
    logic transition_demand;
    logic hud_warning;
    logic mrm;
    logic [3:0] td_remain_sec;

    // Simulation parameter overrides to speed up timers
    // Original CLK_FREQ is 100_000_000 (1 sec = 10^8 cycles). We override to 100 (1 sec = 100 cycles)
    localparam int SIM_CLK_FREQ = 100;

    //------------------------------------------------------------
    // 3. DUT Instantiations (Core Logic Pipeline)
    //------------------------------------------------------------
    preprocessor u_preprocessor (
        .clk                (clk),
        .rst_n              (rst_n),
        .timeout_mask_1s    (timeout_mask_1s),
        .consistency_mask_1s_approach_speed (consistency_mask_1s_approach_speed), 
        .consistency_mask_20s_approach_speed (consistency_mask_20s_approach_speed), 
        
        .sensor_data_in     (sensor_data_axi),
        .sim_data_in        (sim_data_axi),
        .sample_seq_in      (sample_seq_axi),
        
        .sensor_data_out    (sensor_data_s1),
        .sim_data_out       (sim_data_s1),
        .processed_data_out (process_data_s1),
        .sample_seq_out     (sample_seq_s1),
        .valid_s1           (valid_s1),
        .clk_cnt            (clk_cnt_s1),
        
        .pred_data_out      (pred_data_s1),
        .prev_processed_data_out (prev_processed_data_out),
        .consistency_mask_1 (consistency_mask_1),
        .consistency_mask_2 (consistency_mask_2),
        .consistency_mask_3 (consistency_mask_3)
    );

    sensor_reliability //
         
    u_sensor_reliability (
        .clk                (clk),
        .rst_n              (rst_n),
        .valid_s1           (valid_s1),
        .sample_seq_in      (sample_seq_s1),
        .clk_cnt            (clk_cnt_s1),
        .sensor_data_in     (sensor_data_s1),
        .processed_data_in  (process_data_s1),
        .prev_processed_data_in (prev_processed_data_out),
        .pred_data_in       (pred_data_s1),
        .consistency_mask_1 (consistency_mask_1),
        .consistency_mask_2 (consistency_mask_2),
        .consistency_mask_3 (consistency_mask_3),
        .situation          (sim_data_s1.situation),
        
        .timeout_mask_1s    (timeout_mask_1s), 
        .consistency_mask_1s_approach_speed (consistency_mask_1s_approach_speed), 
        .consistency_mask_20s_approach_speed (consistency_mask_20s_approach_speed),
        .sample_seq_out_rel (sample_seq_out_rel),
        .valid_out_rel      (valid_out_rel),
        .reliability_out    (reliability_out)
    );

    risk_types u_risk_types (
        .clk                (clk),
        .rst_n              (rst_n),
        .sim_data_in        (sim_data_s1),
        .sensor_data_in     (sensor_data_s1),
        .valid_in           (valid_s1),
        .sample_seq_in      (sample_seq_s1),
        .risk_out           (risk_out_s1),
        .sim_data_out       (sim_data_risk_s1),
        .sensor_data_out    (sensor_data_risk_s1),
        .sample_seq_out     (sample_seq_out_risk_types),
        .valid_out          (valid_out_risk_types)
    );

    risk_control_2 #(
         
    ) u_risk_control (
        .clk                (clk),
        .rst_n              (rst_n),
        .valid_in           (valid_out_risk_types),
        .valid_in_rel       (valid_out_rel),
        .sample_seq_in      (sample_seq_out_risk_types),
        .sample_seq_in_rel  (sample_seq_out_rel),
        .sensor_data_in     (sensor_data_risk_s1),
        .sim_data_in        (sim_data_risk_s1),
        .rel_in             (reliability_out),
        .risk_in            (risk_out_s1),
        
        .sample_seq_out_risk(sample_seq_risk),
        .sample_seq_out_rel (sample_seq_rel),
        .valid_out_rel_risk (valid_rel_risk),
        .risk_out           (risk_out),
        .rel_out            (rel_out),
        .sensor_data_out    (sensor_data_out),
        .sim_data_out       (sim_data_out),
        .transition_demand  (transition_demand),
        .hud_warning        (hud_warning),
        .mrm                (mrm),
        .td_remain_sec      (td_remain_sec)
    );

    //------------------------------------------------------------
    // 4. Test Scenario Runner
    //------------------------------------------------------------
    initial begin
        // Initialize
        sensor_data_axi = '0;
        sim_data_axi = '0;
        sample_seq_axi = 0;
        
        $display("[TB] System Reset...");
        rst_n = 0;
        #20;
        rst_n = 1;
        $display("[TB] Reset Released. Wait for pipeline...");
        
        // Wait some cycles
        #50;
        
        // Scenario: Generate 1 sample every (SIM_CLK_FREQ/10) cycles to simulate 10Hz sampling.
        for (int i = 0; i < 500; i++) begin
            sample_seq_axi = sample_seq_axi + 1;
            
            // Stuck sensor scenario to test 20s stuck mask
            if (i > 20) begin
                sensor_data_axi.distance = 100;
                sensor_data_axi.approach_speed = 50;
            end else begin
                sensor_data_axi.distance = i * 10;
                sensor_data_axi.approach_speed = i * 5;
            end
            
            sim_data_axi.speed_x = i * 20;
            sensor_data_axi.accel_x = 0;
            sim_data_axi.manual_mode = 0; // Auto mode
            
            @(posedge clk);
            
            // Wait for 10 cycles (10Hz sampling if SIM_CLK_FREQ=100)
            repeat (SIM_CLK_FREQ / 10) @(posedge clk);
        end
        
        #500;
        $display("[TB] Simulation Completed!");
        $finish;
    end

    //------------------------------------------------------------
    // 5. Output Monitoring
    //------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && valid_rel_risk != 0) begin
            $display("[%0t] SEQ: %0d | Dist_out: %0d | MRM: %b | TD_remain: %0d", 
                     $time, sample_seq_risk, sensor_data_out.distance, 
                     mrm, td_remain_sec);
        end
    end

endmodule
