`timescale 1ns / 1ps

import types_pkg::*;

// Replays the exact REG0..REG9 images captured by pl_verification_logger.py.
// This testbench does not recreate or reinterpret the CARLA floating-point
// values: the same 32-bit words sent to the PS are written over AXI4-Lite.
module tb_carla_axi_replay;

    localparam integer AXI_ADDR_W = 6;
    localparam integer SIM_CLK_FREQ_HZ = 2000;
    localparam integer SAMPLE_RATE_HZ = 20;
    localparam integer NOMINAL_GAP_CYCLES = SIM_CLK_FREQ_HZ / SAMPLE_RATE_HZ;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n;
    logic [AXI_ADDR_W-1:0] awaddr, araddr;
    logic [2:0] awprot, arprot;
    logic awvalid, awready;
    logic [31:0] wdata;
    logic [3:0] wstrb;
    logic wvalid, wready;
    logic [1:0] bresp;
    logic bvalid, bready;
    logic arvalid, arready;
    logic [31:0] rdata;
    logic [1:0] rresp;
    logic rvalid, rready;

    integer fail_count = 0;
    integer replay_count = 0;
    longint unsigned cycle_count = 0;
    longint unsigned last_commit_cycle = 0;

    always @(posedge clk)
        cycle_count <= cycle_count + 1;

    top_controller #(
        .C_S_AXI_DATA_WIDTH(32),
        .C_S_AXI_ADDR_WIDTH(AXI_ADDR_W),
        .CLK_FREQ_HZ(SIM_CLK_FREQ_HZ),
        .SAMPLE_RATE_HZ(SAMPLE_RATE_HZ)
    ) dut (
        .S_AXI_ACLK(clk), .S_AXI_ARESETN(rst_n),
        .S_AXI_AWADDR(awaddr), .S_AXI_AWPROT(awprot),
        .S_AXI_AWVALID(awvalid), .S_AXI_AWREADY(awready),
        .S_AXI_WDATA(wdata), .S_AXI_WSTRB(wstrb),
        .S_AXI_WVALID(wvalid), .S_AXI_WREADY(wready),
        .S_AXI_BRESP(bresp), .S_AXI_BVALID(bvalid), .S_AXI_BREADY(bready),
        .S_AXI_ARADDR(araddr), .S_AXI_ARPROT(arprot),
        .S_AXI_ARVALID(arvalid), .S_AXI_ARREADY(arready),
        .S_AXI_RDATA(rdata), .S_AXI_RRESP(rresp),
        .S_AXI_RVALID(rvalid), .S_AXI_RREADY(rready)
    );

    task automatic axi_write(
        input logic [AXI_ADDR_W-1:0] addr,
        input logic [31:0] data
    );
        integer guard;
        begin
            @(negedge clk);
            awaddr = addr;
            awvalid = 1'b1;
            wdata = data;
            wstrb = 4'hF;
            wvalid = 1'b1;
            guard = 0;
            while (!(awready && wready) && guard < 30) begin
                @(posedge clk);
                #1;
                guard = guard + 1;
            end
            if (guard >= 30) begin
                fail_count = fail_count + 1;
                $display("[FAIL] AXI write handshake timeout addr=0x%0h", addr);
            end
            // Xilinx AXI template READY is registered. Keep VALID asserted
            // through the edge at which the transfer is sampled.
            @(posedge clk);
            #1;
            @(negedge clk);
            awvalid = 1'b0;
            wvalid = 1'b0;
            bready = 1'b1;
            guard = 0;
            while (!bvalid && guard < 30) begin
                @(posedge clk);
                #1;
                guard = guard + 1;
            end
            if (guard >= 30 || bresp !== 2'b00) begin
                fail_count = fail_count + 1;
                $display("[FAIL] AXI write response addr=0x%0h bresp=%b", addr, bresp);
            end
            @(negedge clk);
            bready = 1'b0;
        end
    endtask

    task automatic wait_for_pipeline(input logic [31:0] expected_seq);
        integer guard;
        begin
            guard = 0;
            while ((dut.sample_seq_risk !== expected_seq ||
                    dut.sample_seq_rel !== expected_seq) && guard < 100) begin
                @(posedge clk);
                #1;
                guard = guard + 1;
            end
            if (guard >= 100) begin
                fail_count = fail_count + 1;
                $display("[FAIL] pipeline timeout seq=%0d risk_seq=%0d rel_seq=%0d",
                         expected_seq, dut.sample_seq_risk, dut.sample_seq_rel);
            end
        end
    endtask

    task automatic stage_words(input logic [31:0] words [0:9]);
        integer index;
        begin
            // REG0..REG8 are staging registers. REG9 is deliberately omitted
            // here because its write is the atomic new-sample commit.
            for (index = 0; index < 9; index = index + 1)
                axi_write(index * 4, words[index]);
        end
    endtask

    task automatic commit_at_gap(
        input logic [31:0] seq,
        input longint unsigned host_gap_ns
    );
        longint unsigned gap_cycles;
        longint unsigned target_cycle;
        begin
            if (host_gap_ns == 0)
                gap_cycles = NOMINAL_GAP_CYCLES;
            else
                gap_cycles = (host_gap_ns * SIM_CLK_FREQ_HZ + 500000000) / 1000000000;
            if (gap_cycles < 1)
                gap_cycles = 1;

            if (last_commit_cycle != 0) begin
                target_cycle = last_commit_cycle + gap_cycles;
                while (cycle_count < target_cycle)
                    @(posedge clk);
            end
            axi_write(6'h24, seq);
            last_commit_cycle = cycle_count;
        end
    endtask

    initial begin : replay
        integer vector_fd;
        integer result_fd;
        integer parsed;
        integer scan_seq;
        integer line_number;
        longint unsigned host_gap_ns;
        logic [31:0] words [0:9];
        reg [8*2048-1:0] header;
        string vector_file;
        string result_file;

        rst_n = 1'b0;
        awaddr = '0; awprot = '0; awvalid = 1'b0;
        wdata = '0; wstrb = '0; wvalid = 1'b0; bready = 1'b0;
        araddr = '0; arprot = '0; arvalid = 1'b0; rready = 1'b0;

        if (!$value$plusargs("VECTOR_FILE=%s", vector_file))
            vector_file = "pl_vectors.csv";
        if (!$value$plusargs("RESULT_FILE=%s", result_file))
            result_file = "pl_replay_results.csv";

        vector_fd = $fopen(vector_file, "r");
        if (vector_fd == 0) begin
            $display("[FAIL] cannot open vector file: %s", vector_file);
            fail_count = fail_count + 1;
            $finish;
        end
        result_fd = $fopen(result_file, "w");
        if (result_fd == 0) begin
            $display("[FAIL] cannot open result file: %s", result_file);
            fail_count = fail_count + 1;
            $finish;
        end
        $fwrite(result_fd,
            "sample_seq,commit_cycle,risk_word,reliability_word,risk_seq,rel_seq,command,status_speed_limit,range_mask,jump_mask,stuck_mask,noise_mask,consistency_mask,timeout_mask,cons_distance,cons_approach,cons_accel_x,cons_accel_y,cons_accel_z,cons_gyro_x,cons_gyro_y,cons_gyro_z,sensor_gyro_z,pred_gyro_z_1,pred_gyro_z_2,pred_gyro_z_3,delta_gyro_x,delta_gyro_y,delta_incline_x,delta_incline_y,stuck_cnt_gyro_x,stuck_cnt_gyro_y,timeout_cnt_gyro_x,timeout_hold_gyro_x,raw_stuck_gyro_x,cond_b_gyro_x,testable_gyro_x,valid_s1,timeout_mask1_gyro_x\n");

        parsed = $fgets(header, vector_fd);
        repeat (6) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        line_number = 1;

        while (!$feof(vector_fd)) begin
            parsed = $fscanf(vector_fd,
                "%d,%d,%h,%h,%h,%h,%h,%h,%h,%h,%h,%h\n",
                scan_seq, host_gap_ns,
                words[0], words[1], words[2], words[3], words[4],
                words[5], words[6], words[7], words[8], words[9]);
            line_number = line_number + 1;
            if (parsed == 12) begin
                if (words[9] !== scan_seq[31:0]) begin
                    fail_count = fail_count + 1;
                    $display("[FAIL] line=%0d CSV seq=%0d REG9=%0d",
                             line_number, scan_seq, words[9]);
                end

                stage_words(words);
                commit_at_gap(words[9], host_gap_ns);
                wait_for_pipeline(words[9]);

                if ($isunknown({dut.u_axi_slave.read_reg9,
                                dut.u_axi_slave.read_reg10,
                                dut.u_axi_slave.read_reg11,
                                dut.u_axi_slave.read_reg12,
                                dut.u_axi_slave.read_reg13,
                                dut.u_axi_slave.read_reg14})) begin
                    fail_count = fail_count + 1;
                    $display("[FAIL] unknown output bit seq=%0d", words[9]);
                end
                if (dut.u_axi_slave.read_reg11 !== words[9] ||
                    dut.u_axi_slave.read_reg12 !== words[9]) begin
                    fail_count = fail_count + 1;
                    $display("[FAIL] output sequence mismatch input=%0d risk=%0d rel=%0d",
                             words[9], dut.u_axi_slave.read_reg11,
                             dut.u_axi_slave.read_reg12);
                end
                if (dut.u_axi_slave.read_reg9[17:16] !== 2'b11) begin
                    fail_count = fail_count + 1;
                    $display("[FAIL] output valid mismatch seq=%0d valid=%b",
                             words[9], dut.u_axi_slave.read_reg9[17:16]);
                end

                $fwrite(result_fd, "%0d,%0d,%08h,%08h,%0d,%0d,%08h,%08h,%03h,%03h,%03h,%03h,%03h,%03h,%01h,%01h,%01h,%01h,%01h,%01h,%01h,%01h,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                        words[9], last_commit_cycle,
                        dut.u_axi_slave.read_reg9,
                        dut.u_axi_slave.read_reg10,
                        dut.u_axi_slave.read_reg11,
                        dut.u_axi_slave.read_reg12,
                        dut.u_axi_slave.read_reg13,
                        dut.u_axi_slave.read_reg14,
                        dut.u_sensor_reliability.range_err,
                        dut.u_sensor_reliability.jump_err,
                        dut.u_sensor_reliability.stuck_err,
                        dut.u_sensor_reliability.noise_err,
                        dut.u_sensor_reliability.cons_err,
                        dut.u_sensor_reliability.timeout_err,
                        dut.u_sensor_reliability.cons_err_distance,
                        dut.u_sensor_reliability.cons_err_approach_speed,
                        dut.u_sensor_reliability.cons_err_accel_x,
                        dut.u_sensor_reliability.cons_err_accel_y,
                        dut.u_sensor_reliability.cons_err_accel_z,
                        dut.u_sensor_reliability.cons_err_gyro_x,
                        dut.u_sensor_reliability.cons_err_gyro_y,
                        dut.u_sensor_reliability.cons_err_gyro_z,
                        $signed(dut.sensor_data_s1.gyro_z),
                        $signed(dut.pred_data_s1.pred_gyro_z_1),
                        $signed(dut.pred_data_s1.pred_gyro_z_2),
                        $signed(dut.pred_data_s1.pred_gyro_z_3),
                        $signed(dut.process_data_s1.delta_gyro_x),
                        $signed(dut.process_data_s1.delta_gyro_y),
                        $signed(dut.process_data_s1.delta_incline_x),
                        $signed(dut.process_data_s1.delta_incline_y),
                        dut.u_sensor_reliability.u_chk_gyro_x.stuck_cnt,
                        dut.u_sensor_reliability.u_chk_gyro_y.stuck_cnt,
                        dut.u_sensor_reliability.u_chk_gyro_x.timeout_cnt,
                        dut.u_sensor_reliability.u_chk_gyro_x.timeout_confirm_hold,
                        dut.u_sensor_reliability.u_chk_gyro_x.raw_stuck,
                        dut.u_sensor_reliability.u_chk_gyro_x.cond_b,
                        dut.u_sensor_reliability.u_chk_gyro_x.testable,
                        dut.valid_s1,
                        dut.u_sensor_reliability.u_chk_gyro_x.timeout_mask_1s);
                replay_count = replay_count + 1;
            end else if (parsed != -1) begin
                fail_count = fail_count + 1;
                $display("[FAIL] malformed vector line=%0d fields=%0d", line_number, parsed);
            end
        end

        $fclose(vector_fd);
        $fclose(result_fd);
        $display("CARLA AXI REPLAY RESULT: SAMPLES=%0d FAIL=%0d", replay_count, fail_count);
        $finish;
    end

endmodule
