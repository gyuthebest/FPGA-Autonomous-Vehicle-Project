// 단위 맞는지 확인 필요 (거리: m, 속도: km/h, 가속도: LSB) <- 일단 이 기준으로 작성함

module motion_predictor #(
	// 단위 변환 및 SAMPLE_PERIOD 반영된 상수
	// dt = sample period 50ms로 일단 임의 계산 (추후 변경) / 8192LSB = 1g = 9.81m/s^2

	// 9.81 x 0.05(=50ms) x 3.6/8192 = 0.0002155
	// 100000[km->cm] x 0.05[dt] / 3600[h->s] = 1.388;
	// 고정 소숫점 활용하기 위한 상수로 지정 1.388 x 2^16 = 91022
	parameter integer SPEED_DISTANCE_CALC = 91022,

	// 0.5 x 981 x 0.05 x 0.05 / 8192 = 0.000149
	// 고정 소숫점 활용하기 위한 상수로 지정 0.000149 x 2^16 = 9.81 -> 10으로 근사
	parameter integer ACCEL_DISTANCE_CALC = 10

)(
	input logic clk,
	input logic rst_n,
	input logic valid_s1,

	input sensor_data_t sensor_data,

	output logic signed [15:0] expected_distance,
	output logic valid_s2
);

	logic [14:0] prev_distance;
	logic [7:0]	prev_speed;
	logic signed [15:0] prev_accel;

	logic first_sample;

	logic signed [31:0] speed_distance;	// 중간 계산 변수
	logic signed [31:0] accel_distance;

	
	always_ff @(posedge clk) begin
		if(!rst_n) begin

			prev_distance <= '0;
			prev_speed	<= '0;
			prev_accel <= '0;
			expected_distance <= '0;
			speed_distance <= '0;
			accel_distance <= '0;
			valid_s2 <= 1'b0;

			first_sample <= 1'b1;
		end
		else begin

			if(valid_s1) begin
				// 이전값으로 저장
				prev_distance <= sensor_data.distance;
				prev_speed		<= sensor_data.speed;
				prev_accel		<= sensor_data.accel_x;

				if(first_sample) begin
					first_sample <= 1'b0;
					valid_s2 <= 1'b0;		// stage2도 첫샘플 처리
				end

				else begin
					//중간 계산
					speed_distance <= (prev_speed * SPEED_DISTANCE_CALC) >>> 16;
					accel_distance <= (prev_accel * ACCEL_DISTANCE_CALC) >>> 16;
				
					valid_s2 <= 1'b1;
				end
			end
			
			else
				valid_s2 <= 1'b0;

			if(valid_s2)
			expected_distance <= prev_distance - speed_distance - accel_distance;
		end
	
	end


endmodule
