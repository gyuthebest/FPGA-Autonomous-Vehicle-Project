//sensor_checker- �ּ� + ������
//abs �� �ް� �ȿ��� ����ϰ� ������


`timescale 1ns / 1ps

import types_pkg::*;



// range check

module range_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD_MAX = 100,
    parameter THRESHOLD_MIN = 100,
    parameter USE_MIN = 1'b1,
    parameter USE_MAX = 1'b1,
    parameter int unsigned U = 1,    // ← unsigned
    parameter int unsigned D = 1,
    parameter int unsigned N = 10
)( 
    input clk,
    input rst_n,
    input valid_s1,
    input signed [WIDTH-1:0] sensor_data,
    output logic range_error
    );

    logic raw_range;
    logic [$clog2(N+1)-1:0] range_cnt;

    always_comb begin
        raw_range = 1b'0;
        if (valid_s1)
            raw_range = (USE_MIN && (sensor_data < THRESHOLD_MIN)) || (USE_MAX && (sensor_data > THRESHOLD_MAX));
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            range_cnt <= '0;
            range_error <= 1'b0;
        end
        else begin
            if(valid_s1) begin
                range_cnt <= (raw_range == 1'b1) ? ((range_cnt + U > N) ? N : N+U) : ((range_cnt < D) ? '0 : range_cnt - D);
                range_error <= (range_cnt == N) ? 1'b1 : 1'b0;
            end
        end
    end

endmodule


// jump check

module jump_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD = 100,
    parameter int unsigned U = 1,    // ← unsigned
    parameter int unsigned D = 1,
    parameter int unsigned N = 10
)(
    input clk,
    input rst_n,
    input valid_s1,
    input timeout_mask_2s,
    input signed [WIDTH-1:0] processed_data,
    input signed [WIDTH-1:0] prev_processed_data,
    input [1:0] situation,
    output logic jump_error
    );

    logic [1:0] raw_jump;
    logic [$clog2(N+1)-1:0] jump_cnt;

    assign jump_error = (jump_cnt >= N);

    always_comb begin
        raw_jump = 2'b00;
        if (valid_s1) begin
            if (timeout_mask_2s) raw_jump = 2'b10;
            else begin
                if (processed_data - prev_processed_data >= -THRESHOLD && processed_data - prev_processed_data <= THRESHOLD) raw_jump = 2'b00;
                else if ((situation == 2'b01) || (situation == 2'b10)) raw_jump = 2'b10;
                else raw_jump = 2'b01;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            jump_cnt <= '0;  
        end
        else begin
            if (valid_s1) begin
                if (raw_stuck == 2'b00) stuck_cnt <= (stuck_cnt < D) ? '0 : stuck_cnt - D;
                else if (raw_stuck == 2'b01) stuck_cnt <= stuck_cnt + U;
                else stuck_cnt <= stuck_cnt;
            end
        end
    end
endmodule

// stuck check

module stuck_check #(
    parameter WIDTH        = 16,
    parameter THRESHOLD    = 0,    // ← signed 유지
    parameter CHANNEL_TYPE = 1,
    parameter int unsigned U = 1,    // ← unsigned
    parameter int unsigned D = 1,
    parameter int unsigned N = 10
)(
    input clk,
    input rst_n,
    input valid_s1,
    input timeout_mask_1s,
    input stuck_mask,
    input signed [WIDTH-1:0] processed_data,
    input signed [TW-1:0] trig_val_1,
    input signed [TW-1:0] trig_val_2,
    output logic stuck_error
    );
    
    logic [1:0] raw_stuck;
    logic cond_b;
    logic [$clog2(N+1)-1:0] stuck_cnt;
    logic testable;

    assign stuck_error = (stuck_cnt >= N);

    always_comb begin
        case (CHANNEL_TYPE)
            1:  cond_b = (trig_val_1 >  THRESHOLD) || (trig_val_1 < -THRESHOLD);
            2:  cond_b = (trig_val_1 - trig_val_2 >  THRESHOLD) || (trig_val_1 - trig_val_2 < -THRESHOLD);
            default: cond_b = 1'b1;
        endcase

        testable = con_b & ~stuck_mask;
        
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
                if (raw_stuck == 2'b00) stuck_cnt <= (stuck_cnt < D) ? '0 : stuck_cnt - D;
                else if (raw_stuck == 2'b01) stuck_cnt <= stuck_cnt + U;
                else stuck_cnt <= stuck_cnt;
            end
        end
    end
endmodule

// timeout Check
module timeout_check #(
    parameter UPDATE_CLK_X2 = 20,    // ← signed 유지
    parameter int unsigned U = 1,    // ← unsigned
    parameter int unsigned D = 1,
    parameter int unsigned N = 10,
    parameter DROP_N = 2
)(
    input clk,
    input rst_n,
    input [31:0] clk_cnt,
    input valid_s1,
    output logic timeout_mask_1s,
    output logic timeout_mask_2s,
    output logic timeout_error
);

    logic [$clog2(N+1)-1:0] timeout_cnt;
    logic [1:0] timeout_drop;

    wire raw_timeout = (clk_cnt == UPDATE_CLK_X2 - 1);
    assign timeout_error = (timeout_cnt >= N);

    assign timeout_mask_1s = (timeout_drop == DROP_N);   // 복구 첫 샘플만
    assign timeout_mask_2s = (timeout_drop != 0);        // 복구 첫·둘째 샘플

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            timeout_cnt <= '0;
            timeout_mask <= 1'b0;
        end
        else begin
            if (raw_timeout) timeout_drop <= DROP_N;
            else if (valid_s1 && timeout_drop != 0) timeout_drop <= timeout_drop - 1;

            if (raw_timeout == 1'b1) timeout_cnt <= timeout_cnt + U;
            else timeout_cnt <= (timeout_cnt < D) ? '0 : timeout_cnt - D;
        end
    end
endmodule


//noise_check

module noise_check #(
    parameter WIDTH        = 16,
    parameter HISTORY      = 10,
    parameter THRESHOLD_1  = 10,
    parameter THRESHOLD_2  = 10,
    parameter CHANNEL_TYPE = 1,
    parameter int unsigned U = 1,    // ← unsigned
    parameter int unsigned D = 1,
    parameter int unsigned N = 10
)(
    input logic clk,
    input logic rst_n,
    input logic jump_error,
    input logic valid_s1,
    output logic noise_error
    );

    logic 
    
    always_ff @(posedge clk) begin // ����: ���⸮��
    if (!rst_n) begin
       noise_history <= '0;
      end
    else if (new_sample) begin
        noise_history <= {
            noise_history[HISTORY-2:0],jump_error};
    end
    end
//�Ʊ� stuck check �� ������ ������    
always_comb begin
    noise_count = 0;

    for (int i = 0; i < HISTORY; i++) begin
        noise_count = noise_count + noise_history[i];
    end
end
always_comb begin
    noise_high = (noise_count >= 4);
end

endmodule