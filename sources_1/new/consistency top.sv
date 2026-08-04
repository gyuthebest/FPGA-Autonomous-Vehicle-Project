module consistency_top(

	input logic clk,
	input logic rst_n,
	input logic valid_s1,

	input sensor_data_t sensor_data,

	output each_reliability_t consistency_state
);

	//wire 선언
	logic signed [15:0] expected_distance;
	logic signed [15:0] expected_speed;
	logic signed [15:0] expected_gyro;

	logic motion_valid_s2;
	logic velocity_valid_s2;
	logic lateral_valid_s2;

	logic signed [15:0] distance_residual;
	logic signed [15:0] speed_residual;
	logic signed [15:0] gyro_residual;

	logic distance_flag;
	logic speed_flag;
	logic gyro_flag;

	logic motion_error;
	logic velocity_error;
	logic lateral_error;

	logic [15:0] distance_threshold;
	logic [15:0] speed_threshold;
	logic [15:0] gyro_threshold;

	// Threshold
	// Distance threshold
	// Euro NCAP 주행 환경 분류에 따라 구간 설정 (도심(<30), 일반도로(30-80), 고속도로(>80))
	// 샘플링 주기(50ms로 우선 설정함)동안 차량이 이동하는 거리의 약 5%를 기준으로 설정함
	// 30km/h -> 2cm / 80km/h -> 6cm / 130km/h -> 9cm
	always_comb begin
		if (sensor_data.speed <30)
			distance_threshold = 16'd2;
		else if (sensor_data.speed <80)
			distance_threshold = 16'd6;
		else
			distance_threshold = 16'd9;
	end

	// Speed threshold
	// distance와 달리 speed나 accel에 따라 threshold 변경할 근거를 아직 못 찾아서 일단 고정 threshold로 설정함
	// CARLA 쓰니까 정상 주행 데이터의 Residual 분포 확인해서 설정하면 될 듯 (다만 지금 저장공간 때문에 내 컴퓨터에 안 깔려서 아직 못했음)
			assign speed_threshold = 16'd3;

	// Gyro threshold
	// Steering이 커질수록 bicycle model의 단순화 오차, 타이어 슬립각, 비선형성, Ackermann 오차 증가
	always_comb begin
		if (sensor_data.steering < 5)		// 완만 (실제 각도 10도 내외)
			gyro_threshold = 16'd2
		else if (sensor_data.steering < 15)	// 일반 회전 (실제 각도 30도 내외)
			gyro_threshold = 16'd4
		else								// 급조향
			gyro_threshold = 16'd6
	end

	// Motion predictor
	motion_predictor u_motion_predictor (
		.clk(clk),
		.rst_n(rst_n),
		.valid_s1(valid_s1),
		.sensor_data(sensor_data),
		.expected_distance(expected_distance),
		.valid_s2(motion_valid_s2)
	);

	// Velocity predictor
	velocity_predictor u_velocity_predictor (
		.clk(clk),
		.rst_n(rst_n),
		.valid_s1(valid_s1),
		.sensor_data(sensor_data),
		.expected_speed(expected_speed),
		.valid_s2(velocity_valid_s2)
	);

	// Lateral predictor
	lateral_predictor u_lateral_predictor (
		.clk(clk),
		.rst_n(rst_n),
		.valid_s1(valid_s1),
		.sensor_data(sensor_data),
		.expected_gyro(expected_gyro),
		.valid_s2(lateral_valid_s2)
	);

	//-----------------------------------------------------------------
	// Common checker 연결
	//-----------------------------------------------------------------

	// distance checker
	common_checker u_distance_checker (

		.clk(clk),
		.rst_n(rst_n),
		.valid_s2(motion_valid_s2),

		.measured_value(sensor_data.distance),
		.expected_value(expected_distance),
		.threshold(distance_threshold),

		.residual(distance_residual),
		.residual_flag(distance_flag),
		.error(motion_error)

	);

	// speed checker
	common_checker u_speed_checker (

		.clk(clk),
		.rst_n(rst_n),
		.valid_s2(velocity_valid_s2),

		.measured_value(sensor_data.speed),
		.expected_value(expected_speed),
		.threshold(speed_threshold),

		.residual(speed_residual),
		.residual_flag(speed_flag),
		.error(velocity_error)

	);

	// gyro checker
	common_checker u_gyro_checker (

		.clk(clk),
		.rst_n(rst_n),
		.valid_s2(lateral_valid_s2),

		.measured_value(sensor_data.gyro_z),
		.expected_value(expected_gyro),
		.threshold(gyro_threshold),

		.residual(gyro_residual),
		.residual_flag(gyro_flag),
		.error(lateral_error)

	);


	//-----------------------------------------------------------------
	// Score Adjust 연결
	//-----------------------------------------------------------------

	reliability_decision u_reliability_decision(
		.clk(clk),
		.rst_n(rst_n),
		.valid_s2(motion_valid_s2),		// 각 predictor의 valid_s2는 valid_s1에 따라 출력되므로 같은 타이밍일 것 같아서 대표 하나로 연결함

		.motion_error(motion_error),
		.velocity_error(velocity_error),
		.lateral_error(lateral_error),

		.consistency_state(consistency_state)

	);

endmodule


//=========================================
// Reliability Decision Module
//=========================================

module reliability_decision(
	input logic clk,
	input logic rst_n,
	input logic valid_s2,

	input logic motion_error,
	input logic velocity_error,
	input logic lateral_error,

	output each_reliability_t consistency_state
);

	logic [1:0] error_count;
	logic [1:0] hold_count;
	each_reliability_t current_state;	// motion, velocity, lateral 고려한 consistency 전체 신뢰도. 최소 유지 시간 반영되지 않은 상태

	always_comb begin			// 각 error 가중치 없이 동일하게 봄
		error_count = motion_error + velocity_error + lateral_error;
	end

	always_comb begin
		if(error_count == 0)
			current_state = STATE_NORMAL;
		else if(error_count == 1)
			current_state = STATE_DEGRADED;
		else
			current_state = STATE_UNAVAILABLE;
	end

	always_ff @(posedge clk) begin
		if(!rst_n) begin
			consistency_state <= STATE_NORMAL;
			hold_count <= '0;
		end
		else if(valid_s2) begin		// 샘플 주기가 50ms인가? 그래서 150ms-> count3으로 구현함상태가 같거나 degrade되는 경우는 바로 변경
			if(current_state >= consistency_state) begin
				consistency_state <= current_state;
				hold_count <= '0;
			end
			else begin				// 상태가 좋아지는 경우 최소 유지 시간에 따라 변경
				if(hold_count <3)
					hold_count <= hold_count +1;
				else begin
					consistency_state <= current_state;
					hold_count <= '0;
				end
			end
		end
				

endmodule
