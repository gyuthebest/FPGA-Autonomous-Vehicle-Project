`timescale 1ns/1ps
import types_pkg::*;

module tb_distance_debug;
    logic clk = 0, rst_n = 0, valid_s1 = 0;
    sensor_data_t sensor = '0;
    processed_data_t processed = '0, prev_processed = '0;
    pred_data_t pred = '0;
    reliability_state_t reliability;
    logic timeout_mask, m1, m2, valid_out;
    logic [31:0] sample_seq_out;

    always #5 clk = ~clk;

    sensor_reliability #(.CLK_FREQ_HZ(1000), .SAMPLE_RATE_HZ(20)) dut (
        .clk(clk), .rst_n(rst_n), .valid_s1(valid_s1),
        .sample_seq_in(32'd0), .clk_cnt(32'd0),
        .consistency_mask_1(1'b0), .consistency_mask_2(1'b0),
        .consistency_mask_3(1'b0), .situation(3'b000),
        .consistency_mask_1s_approach_speed(m1),
        .timeout_mask_1s(timeout_mask),
        .consistency_mask_20s_approach_speed(m2),
        .sensor_data_in(sensor), .processed_data_in(processed),
        .prev_processed_data_in(prev_processed), .pred_data_in(pred),
        .sample_seq_out_rel(sample_seq_out), .valid_out_rel(valid_out),
        .reliability_out(reliability)
    );

    task sample;
        repeat (49) @(posedge clk);
        valid_s1 <= 1'b1;
        @(posedge clk);
        valid_s1 <= 1'b0;
    endtask

    initial begin
        sensor.distance = 15'd20000;
        pred.pred_distance = 21'sd800000;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (80) sample();
        #1;
        $display("DIST state=%0d range=%0b jump=%0b stuck=%0b noise=%0b cons=%0b timeout=%0b",
            reliability.distance.state, dut.range_err[0], dut.jump_err[0],
            dut.stuck_err[0], dut.noise_err[0], dut.cons_err[0], dut.timeout_err[0]);
        $finish;
    end
endmodule
