//---------------------------------------------------------------------------------------
// Common Checker
//---------------------------------------------------------------------------------------

import types_pkg::*;

module common_checker #(
	
	parameter COUNTER_WIDTH = 2,		// 혹시 consistency check 내에서도 다르게 적용할까봐 일단 파라미터로 둠
	parameter N = 3,
	parameter UP_STEP = 1,
	parameter DOWN_STEP = 1

)(

	input logic clk,
	input logic rst_n,

	input logic valid_s2,

	input logic signed [15:0] measured_value,		// 현재 센서값(k)
	input logic signed [15:0] expected_value,		// Predictor module에서 계산한 값
	input logic [15:0] threshold,

	output logic signed [15:0] residual,			// measured - expected
	output logic residual_flag,
	output logic error
	
);

	// 변수 선언
	logic [15:0] residual_abs;
	logic [COUNTER_WIDTH-1:0] counter;		// consistency check counter 2bit
	

	// Residual Calculation
	always_comb begin
		residual = measured_value - expected_value;

		if(residual < 0)
			residual_abs = -residual;
		else
			residual_abs = residual;

		residual_flag = (residual_abs > threshold);
	end


	// Counter Update
	always_ff @(posedge clk) begin

		// Reset
		if(!rst_n)
			counter <= '0;

		else if(valid_s2) begin
			if(residual_flag) begin
				if(counter != {COUNTER_WIDTH{1'b1}})		// flag 1일 때 최댓값 아니면 +1 (여기서는 1 단위 증가 감소니까 그냥 != 썼음, UP_STEP 더 크면 변경 필요)
					counter <= counter + UP_STEP;
			end

			else begin
				if(counter != 0)							// flag 0일 때 counter 0 아닐 때만 -1 (underflow 방지)
					counter <= counter - DOWN_STEP;
			end
		end
	end


	// Error count
	always_comb begin
		if(counter >= N)
			error = 1'b1;
		else
			error = 1'b0;
	end


endmodule
