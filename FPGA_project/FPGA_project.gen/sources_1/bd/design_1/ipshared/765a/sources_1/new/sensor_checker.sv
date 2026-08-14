`timescale 1ns / 1ps

import types_pkg::*;

module each_sensor_check #(
    parameter WIDTH = 12,
    parameter DW = 13,
    parameter TW = 13,
    parameter CHANNEL_TYPE_1 = 1,
    parameter CHANNEL_TYPE_2 = 1,
    parameter UPDATE_CLK_X2 = 20,
    parameter DROP_N = 2,
    parameter HISTORY = 10,
    parameter RANGE_THRESHOLD_MAX = 100,
    parameter RANGE_THRESHOLD_MIN = 100,
    parameter JUMP_THRESHOLD = 100,
    parameter STUCK_THRESHOLD = 0,
    parameter NOISE_THRESHOLD_1 = 10,
    parameter NOISE_THRESHOLD_2 = 10,
    parameter RANGE_USE_MIN = 1'b1,
    parameter RANGE_USE_MAX = 1'b1,
    parameter int unsigned RANGE_U = 1,    // ← unsigned
    parameter int unsigned RANGE_D = 1,
    parameter int unsigned RANGE_N = 10,
    parameter int unsigned JUMP_U = 1,    // ← unsigned
    parameter int unsigned JUMP_D = 1,
    parameter int unsigned JUMP_N = 10,
    parameter int unsigned STUCK_U = 1,    // ← unsigned
    parameter int unsigned STUCK_D = 1,
    parameter int unsigned STUCK_N = 10,
    parameter int unsigned TIMEOUT_U = 1,    // ← unsigned
    parameter int unsigned TIMEOUT_D = 1,
    parameter int unsigned TIMEOUT_N = 10
)( 
    input clk,
    input rst_n,
    input valid_s1,
    input [31:0] clk_cnt,
    input stuck_mask,
    input signed [WIDTH-1:0] sensor_data,
    input signed [DW-1:0] processed_data,
    input signed [DW-1:0] prev_processed_data,
    input signed [TW-1:0] trig_val_1,
    input signed [TW-1:0] trig_val_2,
    input [2:0] situation,

    output logic range_error,
    output logic jump_error,
    output logic stuck_error,
    output logic timeout_error,
    output logic noise_error,
    output logic timeout_mask_1s,
    output logic timeout_mask_2s
    );

    logic raw_range;
    logic [1:0] raw_jump;
    logic [1:0] raw_stuck;
    logic raw_timeout;
    logic [31:0] range_cnt;
    logic [31:0] jump_cnt;
    logic [31:0] stuck_cnt;
    logic [31:0] timeout_cnt;
    logic jump_mask;
    logic cond_b;
    logic testable;
    logic [1:0] timeout_drop;
    logic [1:0] timeout_confirm_hold;
    localparam int SUM_W = DW + $clog2(HISTORY + 1);
    logic [SUM_W-1:0] delta_sum;
    logic [HISTORY-1:0][DW-1:0] delta_history;
    logic [HISTORY-1:0] flip_history;

    // range check
    assign range_error = (range_cnt >= RANGE_N);

    always_comb begin
        raw_range = 1'b0;
        if (valid_s1)
            raw_range = (RANGE_USE_MIN && (sensor_data < RANGE_THRESHOLD_MIN)) || (RANGE_USE_MAX && (sensor_data > RANGE_THRESHOLD_MAX));
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            range_cnt <= '0;
        end
        else begin
            if(valid_s1) begin
                if (raw_range == 1'b1)
                    range_cnt <= (range_cnt >= RANGE_N - RANGE_U)
                               ? RANGE_N : range_cnt + RANGE_U;
                else
                    range_cnt <= (range_cnt < RANGE_D) ? '0 : range_cnt - RANGE_D;
            end
        end
    end
   
    // jump check
    assign jump_error = (jump_cnt >= JUMP_N);

    always_comb begin
        if ((CHANNEL_TYPE_1 == 1) && ((situation == 3'b001) || (situation == 3'b010))) jump_mask = 1'b1;
        else if ((CHANNEL_TYPE_1 == 2) && (situation == 3'b011)) jump_mask = 1'b1;
        else jump_mask = 1'b0;

        if (valid_s1) begin
            if (timeout_mask_2s) raw_jump = 2'b10;
            else begin
                if (processed_data - prev_processed_data >= -JUMP_THRESHOLD && processed_data - prev_processed_data <= JUMP_THRESHOLD) raw_jump = 2'b00;
                else if (jump_mask) raw_jump = 2'b10;
                else raw_jump = 2'b01;
            end
        end
        else raw_jump = 2'b00;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            jump_cnt <= '0;  
        end
        else begin
            if (valid_s1) begin
                if (raw_jump == 2'b00) jump_cnt <= (jump_cnt < JUMP_D) ? '0 : jump_cnt - JUMP_D;
                else if (raw_jump == 2'b01)
                    jump_cnt <= (jump_cnt >= JUMP_N - JUMP_U)
                              ? JUMP_N : jump_cnt + JUMP_U;
                else jump_cnt <= jump_cnt;
            end
        end
    end
    
    // stuck check
    assign stuck_error = (stuck_cnt >= STUCK_N);

    always_comb begin
        case (CHANNEL_TYPE_2)
            1:  cond_b = (trig_val_1 >= STUCK_THRESHOLD) || (trig_val_1 <= -STUCK_THRESHOLD);
            2:  cond_b = (trig_val_1 - trig_val_2 >= STUCK_THRESHOLD) || (trig_val_1 - trig_val_2 <= -STUCK_THRESHOLD);
            default: cond_b = 1'b1;
        endcase

        testable = cond_b & ~stuck_mask;
        
        if (valid_s1) begin
            if (timeout_mask_1s) raw_stuck = 2'b10;
            else begin
                if (processed_data != 0) raw_stuck = 2'b00;
                else if (testable == 2'b1) raw_stuck = 2'b01;
                else raw_stuck = 2'b10;
            end
        end
        else raw_stuck = 2'b00;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            stuck_cnt   <= '0;
        end
        else begin
            if (valid_s1) begin
                if (raw_stuck == 2'b00) stuck_cnt <= (stuck_cnt < STUCK_D) ? '0 : stuck_cnt - STUCK_D;
                else if (raw_stuck == 2'b01)
                    stuck_cnt <= (stuck_cnt >= STUCK_N - STUCK_U)
                               ? STUCK_N : stuck_cnt + STUCK_U;
                else stuck_cnt <= stuck_cnt;
            end
        end
    end
    
    // timeout check
    // clk_cnt is a shared periodic phase counter.  Gate with valid_s1 so a
    // sample arriving exactly on the boundary heals instead of adding fault
    // evidence.
    assign raw_timeout = !valid_s1 && (clk_cnt == UPDATE_CLK_X2 - 1);
    // Preserve a confirmed timeout across the first recovery samples.  The
    // PS cannot read PL status while no UDP packet is present; without this
    // hold, the counter healed before the first returned CARLA frame could
    // observe the fault.
    assign timeout_error = (timeout_cnt >= TIMEOUT_N) ||
                           (timeout_confirm_hold != 0);

    assign timeout_mask_1s = (timeout_drop == DROP_N);   // 복구 첫 샘플만
    assign timeout_mask_2s = (timeout_drop != 0);        // 복구 첫·둘째 샘플

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            timeout_cnt <= '0;
            timeout_drop <= '0;
            timeout_confirm_hold <= '0;
        end
        else begin
            if (raw_timeout) timeout_drop <= DROP_N;
            else if (valid_s1 && timeout_drop != 0) timeout_drop <= timeout_drop - 1;

            if (raw_timeout && (timeout_cnt >= TIMEOUT_N - TIMEOUT_U))
                timeout_confirm_hold <= DROP_N;
            else if (valid_s1 && timeout_confirm_hold != 0)
                timeout_confirm_hold <= timeout_confirm_hold - 1;

            // Saturate the diagnostic counter at its confirmation threshold.
            // Without saturation, a board left idle accumulates an arbitrarily
            // large count and needs seconds or minutes of valid samples to heal.
            // The timeout decision still confirms at TIMEOUT_N, while recovery
            // is bounded to ceil(TIMEOUT_N / TIMEOUT_D) valid samples.
            if (raw_timeout == 1'b1)
                timeout_cnt <= (timeout_cnt >= TIMEOUT_N - TIMEOUT_U)
                             ? TIMEOUT_N : timeout_cnt + TIMEOUT_U;
            else if (valid_s1) timeout_cnt <= (timeout_cnt < TIMEOUT_D) ? '0 : timeout_cnt - TIMEOUT_D; //valid_s1 추가
        end
    end

    // noise_check
    always_comb begin
        delta_sum = '0;
        for (int i=0; i<HISTORY; i++) delta_sum += delta_history[i];
        noise_error = (delta_sum > NOISE_THRESHOLD_1 * HISTORY) || ($countones(flip_history) > NOISE_THRESHOLD_2);
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            delta_history       <= '0;
            flip_history        <= '0;
        end
        else begin
            if (valid_s1) begin
                // Skip only the first sample after a timeout because its
                // delta spans the missing interval; collect normal samples.
                if (!timeout_mask_1s) begin
                    delta_history <= {delta_history[HISTORY-2:0],(processed_data[DW-1] ? -processed_data : processed_data)};
                    flip_history  <= {flip_history[HISTORY-2:0],(prev_processed_data[DW-1] != processed_data[DW-1])};
                end
            end    
        end
    end

endmodule
