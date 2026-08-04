module velocity_predictor #(
	// 단위 변환(accel를 km/h2로) 및 SAMPLE_PERIOD 반영된 상수
	// dt = sample period 50ms로 일단 임의 계산 (추후 변경) / 8192LSB = 1g = 9.81m/s^2 / 1m/s = 3.6km/h
	// 9.81 x 0.05(=50ms) x 3.6/8192 = 0.0002155
	// 고정 소숫점 활용하기 위한 상수로 지정 0.00021555 x 2^16 = 14
	parameter integer DELTA_SPEED_CALC = 14
)(
	input logic clk,
	input logic rst_n,
	input logic valid_s1,			// 전처리 후 받는 유효 센서값

	input sensor_data_t sensor_data,
	
	output logic signed [15:0] expected_speed,
	output logic valid_s2	// valid_s1에 따라 다음 클럭 s2 본계산 수행 여부 전달
);

	logic [7:0] prev_speed;
	logic signed [15:0] prev_accel;		// 이전값 저장

	logic first_sample;

	logic signed [31:0] delta_speed;	// 중간 계산 변수

	// S1, S2로 2클럭짜리 계산
	// S1은 이전값 저장 및 중간 계산, S2는 본계산
	always_ff @(posedge clk) begin
		if(!rst_n) begin
			prev_speed	<= '0;
			prev_accel <= '0;
			expected_speed <= '0;
			delta_speed <= '0;
			valid_s2 <= 1'b0;

			first_sample <= 1'b1;
		end
		else begin

			// S1 이전값 저장 및 중간 계산
			if(valid_s1) begin
				// 이전값으로 저장
				prev_speed		<= sensor_data.speed;
				prev_accel		<= sensor_data.accel_x;

				if(first_sample) begin
					first_sample <= 1'b0;
					valid_s2 <= 1'b0;		// 다음 클럭의 S2도 첫샘플 처리
				end

				else begin
					//중간 계산
					delta_speed <= (prev_accel * DELTA_SPEED_CALC) >>> 16;
				
					valid_s2 <= 1'b1;
				end
			end

			else
				valid_s2 <= 1'b0;

			// S2 본계산 (이전 클럭의 계산 결과를 활용)
			if(valid_s2)
				expected_speed <= prev_speed + delta_speed;
			// valid_s2 = 0인 경우에는 이전 reg 값 유지
		end
	
	end


endmodule