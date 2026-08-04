// bicycle model 사용
// yaw rate = speed x tan(steering angle) / wheelbase
//tan을 steering angle로 근사 (이후 threshold 완화하여 반영)
// steering 입력이 deg라고 생각하고 rad으로 변환해서 활용함, 센서 원시값이면 식 변경해야함.

module lateral_predictor #(
	// 단위 변환 및 wheelbase 반영 상수
	// steering = 70도를 0.233도씩 구간으로 나눔 / 1도 = 0.01745rad / wheelbase 2.7m
	// 0.233 x 0.01745 x (1000 / 3600)[km/h->m/s] / 2.7[wheelbase]= 0.0041894
	// 고정 소숫점 활용하기 위한 상수로 지정 0.00418974 x 2^16 = 275
	parameter integer EXPECTED_GYRO_CALC = 275
)(
	input logic clk,
	input logic rst_n,
	input logic valid_s1,

	input sensor_data_t sensor_data,

	output logic signed [15:0] expected_gyro,
	output logic valid_s2
);

	logic signed [31:0] calc_s0;		// 계산 크므로 여유 비트폭 설정

// 계산에 2클럭 소요
// S0 : 센서값 기반 계산, S1 : 단위 보정 및 wheelbase 계산
always_ff @(posedge clk) begin
	if(!rst_n) begin
		calc_s0 <= '0;
		expected_gyro <= '0;
		valid_s2 <= 1'b0;
	end
	else begin
		valid_s2 <= valid_s1;		// 계산 클럭 +1이라서 유효 신호 연장, 클럭마다 무조건 갱신
		
		// S0 : 유효 센서값일 때만 계산
		if(valid_s1)
		calc_s0 <= sensor_data.speed * sensor_data.steering;
		// S1 : S0 계산값이 있을 때만 계산
		if(valid_s2)
		expected_gyro <= (calc_s0 * EXPECTED_GYRO_CALC) >>> 16;

	end
end


endmodule
