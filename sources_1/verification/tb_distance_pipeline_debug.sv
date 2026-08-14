`timescale 1ns/1ps
import types_pkg::*;

module tb_distance_pipeline_debug;
    logic clk = 0, rst_n = 0;
    sensor_data_t sensor_in = '0, sensor_s1;
    sim_data_t sim_in = '0, sim_s1;
    processed_data_t processed, prev_processed;
    pred_data_t pred;
    reliability_state_t reliability;
    logic [31:0] seq_in = 0, seq_s1, seq_out, clk_cnt;
    logic valid_s1, valid_rel, tm1, cm1, cm2;
    logic mask1, mask2, mask3;
    always #5 clk = ~clk;

    preprocessor pre (
        .clk, .rst_n, .sensor_data_in(sensor_in), .sim_data_in(sim_in),
        .sample_seq_in(seq_in), .timeout_mask_1s(tm1),
        .consistency_mask_1s_approach_speed(cm1),
        .consistency_mask_20s_approach_speed(cm2),
        .sensor_data_out(sensor_s1), .sim_data_out(sim_s1),
        .processed_data_out(processed),
        .prev_processed_data_out(prev_processed), .pred_data_out(pred),
        .valid_s1, .sample_seq_out(seq_s1), .clk_cnt,
        .consistency_mask_1(mask1), .consistency_mask_2(mask2),
        .consistency_mask_3(mask3)
    );

    sensor_reliability #(.CLK_FREQ_HZ(1000), .SAMPLE_RATE_HZ(20)) rel (
        .clk, .rst_n, .valid_s1, .sample_seq_in(seq_s1), .clk_cnt,
        .sensor_data_in(sensor_s1), .processed_data_in(processed),
        .prev_processed_data_in(prev_processed), .pred_data_in(pred),
        .consistency_mask_1(mask1), .consistency_mask_2(mask2),
        .consistency_mask_3(mask3), .situation(sim_s1.situation),
        .timeout_mask_1s(tm1), .consistency_mask_1s_approach_speed(cm1),
        .consistency_mask_20s_approach_speed(cm2),
        .sample_seq_out_rel(seq_out), .valid_out_rel(valid_rel),
        .reliability_out(reliability)
    );

    task sample;
        repeat (50) @(posedge clk);
        seq_in <= seq_in + 1;
    endtask

    initial begin
        sensor_in.distance = 15'd20000;
        sensor_in.accel_z = 12'sd981;
        sensor_in.temperature = 11'sd220;
        sensor_in.humidity = 7'd30;
        sensor_in.lux = 19'd50000;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (80) sample();
        repeat (5) @(posedge clk);
        $display("PIPE dist=%0d pred=%0d delta=%0d prev_delta=%0d state=%0d range=%0b jump=%0b stuck=%0b noise=%0b cons=%0b timeout=%0b",
            sensor_s1.distance, pred.pred_distance, processed.delta_distance,
            prev_processed.delta_distance, reliability.distance.state,
            rel.range_err[0], rel.jump_err[0], rel.stuck_err[0],
            rel.noise_err[0], rel.cons_err[0], rel.timeout_err[0]);
        $finish;
    end
endmodule
