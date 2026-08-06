`timescale 1ns / 1ps

//output package pkg에서 묶어놔야할듯, output port 정의해줘야함
//current value가 [WIDHT-1:0]으로 정의 됨
//delta의 abs를 넣어야할 것임 ㄴㄴ
//noise check 할 때 jump_error 값 필요 --> logic으로 따로 정의해줘야함 
//new_sample 어디서 받는지... 그리고 sensor input을 preprocessed랑 같이 받을 수 있는 timing이 되는지도 확인해줘야함ㅁ
//stuck check 변화가 있으 때의 조건이 필요할 시 모듈로 가서 input delta를 받아와서 if delta>0 일 때의 조건으로 바꾸면 될듯

//수정: 조도, 온도, 습도, 전압에 대해서는 stuck check 하지 않음. 고정된게 정상인 값이기 때문

import types_pkg::*;

module Sensor_reiability(
    input logic clk,
    input logic rst_n,
    input logic valid, // 추가
    input sensor_data_t sensor_data_in,
    input sim_data_t    sim_data_in, 
    input logic [7:0] Speed, // Speed 어디로 오는지 확인해야함
    
    output logic out_in_range_distance, jump_error_distance, is_stuck_distance, noise_high_distance, // output 추가
    output logic out_in_range_approach_speed, jump_error_approach_speed, is_stuck_approach_speed, noise_high_approach_speed,
    output logic out_in_range_accel_x, jump_error_accel_x, is_stuck_accel_x, noise_high_accel_x,
    output logic out_in_range_accel_y, jump_error_accel_y, is_stuck_accel_y, noise_high_accel_y,
    output logic out_in_range_accel_z, jump_error_accel_z, is_stuck_accel_z, noise_high_accel_z,
    output logic out_in_range_gyro_x, jump_error_gyro_x, is_stuck_gyro_x, noise_high_gyro_x,
    output logic out_in_range_gyro_y, jump_error_gyro_y, is_stuck_gyro_y, noise_high_gyro_y,
    output logic out_in_range_gyro_z, jump_error_gyro_z, is_stuck_gyro_z, noise_high_gyro_z,
    output logic out_in_range_lux, jump_error_lux, noise_high_lux,
    output logic out_in_range_temp, jump_error_temp, noise_high_temp,
    output logic out_in_range_hum, jump_error_hum, noise_high_hum,

    output logic is_timeout_distance, is_timeout_approach_speed,
    output logic is_timeout_accel_x, is_timeout_accel_y, is_timeout_accel_z,
    output logic is_timeout_gyro_x, is_timeout_gyro_y, is_timeout_gyro_z,
    output logic is_timeout_lux, is_timeout_temp, is_timeout_hum,

    output logic temperature_warn
    );
  
    
///distance reliability
//거리 stuck check 애매함 

    range_check #(
    .WIDTH (16),
    .THRESHOLD_MAX(20000),
    .THRESHOLD_MIN(0),
    .USE_MIN(0),
    .USE_MAX(1)
    )u_range_distance(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.distance),
    .out_in_range(out_in_range_distance)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(400)
)u_jump_check_distance(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.distance),
    .temp_except(1'b0), // 수정: abs 대신 temp_except
    .jump_error(jump_error_distance)
    );
    
    // 거리 Stuck check 따로 만들기
    logic [7:0] prev_speed;
    
    always_ff @(posedge clk) begin
        if(!rst_n)
            prev_speed <= 8'd0;
        else if (valid)
            prev_speed <= Speed;
    end
    
    logic speed_change; //차량 속도 변화하는지
    assign speed_change = (Speed != prev_speed); //sensor_data_in에 Speed 있는지 확인 필요
    
    logic distance_stuck_check_en;
    assign distance_stuck_check_en = (sensor_data_in.distance != 16'd20000) && speed_change;
    
    
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(1)
)u_stuck_check_distance(  //수정: 거리 신뢰도 판단에 대한 로직 적용
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.distance),
    .new_sample(valid),
    .check_enable(distance_stuck_check_en),
    .is_stuck(is_stuck_distance) // 수정: abs 삭제
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_check_distance(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_distance),
    .new_sample(valid),
    .noise_high(noise_high_distance)
    );

// 접근속도 reliability. 의문: 확인필요
    range_check_signed #(
        .WIDTH(10), 
        .THRESHOLD_MAX(278), 
        .THRESHOLD_MIN(-278), 
        .USE_MIN(1), 
        .USE_MAX(1)
        )u_range_approach_speed (
        .clk(clk), 
        .rst_n(rst_n), 
        .value(sensor_data_in.approach_speed), 
        .out_in_range(out_in_range_approach_speed)
    );

    jump_check #(
        .WIDTH(10), 
        .THRESHOLD(200) //시뮬레이션 통해 확인 필요
    )u_jump_approach_speed (
        .clk(clk), 
        .rst_n(rst_n), 
        .current_data(sensor_data_in.approach_speed), 
        .temp_except(1'b0), 
        .jump_error(jump_error_approach_speed)
    );

    stuck_check #(
        .WIDTH(10), 
        .HISTORY(10), 
        .THRESHOLD(1)
    )u_stuck_approach_speed (
        .clk(clk), 
        .rst_n(rst_n), 
        .new_sample(valid), 
        .current_data(sensor_data_in.approach_speed), // 새 신호 안 만들고 갖다쓴거임.
        .check_enable(distance_stuck_check_en), 
        .is_stuck(is_stuck_approach_speed)
    );

    noise_check #(
        .WIDTH(10), 
        .HISTORY(10)
    )u_noise_approach_speed (
        .clk(clk), 
        .rst_n(rst_n), 
        .jump_error(jump_error_approach_speed), 
        .new_sample(valid), 
        .noise_high(noise_high_approach_speed)
    );



//Accel_x reliability    

    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(16384),
    .THRESHOLD_MIN(-16384),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_accel_x(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.accel_x),
    .out_in_range(out_in_range_accel_x)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(8192)
)u_jump_accel_x(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.accel_x),
    .temp_except(1'b0),
    .jump_error(jump_error_accel_x)
    );
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(232)
)u_stuck_accel_x(
    .clk(clk),
    .rst_n(rst_n),
    .new_sample(valid),
    .current_data(sensor_data_in.accel_x),
    .check_enable(speed_change),
    .is_stuck(is_stuck_accel_x)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_accel_x(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_accel_x),
    .new_sample(valid),
    .noise_high(noise_high_accel_x)
    );

 //accel_y check
    logic [15:0] prev_gyro_z;
    logic direction_change;
    
    always_ff @(posedge clk) begin           //로직 추가
        if(!rst_n)
            prev_gyro_z <= 16'd0;
        else if (valid)
            prev_gyro_z <= sensor_data_in.gyro_z;
    end
    
    assign direction_change = (sensor_data_in.gyro_z != prev_gyro_z);

    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(16384),
    .THRESHOLD_MIN(-16384),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_accel_y(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.accel_y),
    .out_in_range(out_in_range_accel_y)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(8192)
)u_jump_accel_y(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.accel_y),
    .temp_except(1'b0),
    .jump_error(jump_error_accel_y)
    );
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(24)
)u_stuck_accel_y(
    .clk(clk),
    .rst_n(rst_n),
    .new_sample(valid),
    .current_data(sensor_data_in.accel_y),
    .check_enable(direction_change),
    .is_stuck(is_stuck_accel_y)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_accel_y(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_accel_y),
    .new_sample(valid),
    .noise_high(noise_high_accel_y)
    );
    
 //accel_z 
    logic accel_z_stuck_check_en; // 로직 만들어주기
    assign accel_z_stuck_check_en = (Speed != 8'b0);
    
    
    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(16384),
    .THRESHOLD_MIN(-16384),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_accel_z(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.accel_z),
    .out_in_range(out_in_range_accel_z)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(8192)
)u_jump_accel_z(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.accel_z),
    .temp_except(1'b0),
    .jump_error(jump_error_accel_z)
    );
    
    stuck_check #( //속도가 0이 아닐때인 경우 반영해.
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(20)
)u_stuck_accel_z(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.accel_z),
    .new_sample(valid),
    .check_enable (accel_z_stuck_check_en),
    .is_stuck(is_stuck_accel_z)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_accel_z(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_accel_z),
    .new_sample(valid),
    .noise_high(noise_high_accel_z)
    );
    
 //gyro_x

    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(20480),
    .THRESHOLD_MIN(-20480),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_gyro_x(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.gyro_x),
    .out_in_range(out_in_range_gyro_x)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(10240)
)u_jump_gyro_x(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.gyro_x),
    .temp_except(1'b0),
    .jump_error(jump_error_gyro_x)
    );
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(50)
)u_stuck_gyro_x(
    .clk(clk),
    .rst_n(rst_n),
    .new_sample(valid),
    .current_data(sensor_data_in.gyro_x),
    .check_enable(direction_change),
    .is_stuck(is_stuck_gyro_x)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_gyro_x(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_gyro_x),
    .new_sample(valid),
    .noise_high(noise_high_gyro_x)
    );
    
 //gyro_y   

    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(15360),
    .THRESHOLD_MIN(-15360),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_gyro_y(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.gyro_y),
    .out_in_range(out_in_range_gyro_y)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(7680)
)u_jump_gyro_y(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.gyro_y),
    .temp_except(1'b0),
    .jump_error(jump_error_gyro_y)
    );
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(20)
)u_stuck_gyro_y(
    .clk(clk),
    .rst_n(rst_n),
    .new_sample(valid),
     .current_data(sensor_data_in.gyro_y),
    .check_enable(speed_change),
    .is_stuck(is_stuck_gyro_y)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_gyro_y(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_gyro_y),
    .new_sample(valid),
    .noise_high(noise_high_gyro_y)
    );
    
//gyro_z

    range_check_signed #(
    .WIDTH (16),
    .THRESHOLD_MAX(30720),
    .THRESHOLD_MIN(-30720),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_gyro_z(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.gyro_z),
    .out_in_range(out_in_range_gyro_z)
);
   
    jump_check #(
    .WIDTH(16),
    .THRESHOLD(15360)
)u_jump_gyro_z(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.gyro_z),
    .temp_except(1'b0),
    .jump_error(jump_error_gyro_z)
    );
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(16)
)u_stuck_gyro_z(
    .clk(clk),
    .rst_n(rst_n),
    .new_sample(valid),
    .current_data(sensor_data_in.gyro_z),
    .check_enable(direction_change),
    .is_stuck(is_stuck_gyro_z)
    );
    
     noise_check #(
    .WIDTH(16),
    .HISTORY(10)
)u_noise_gyro_z(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_gyro_z),
    .new_sample(valid),
    .noise_high(noise_high_gyro_z)
    );
    
 //lux

    range_check_signed #(
    .WIDTH (19),
    .THRESHOLD_MAX(30720),
    .THRESHOLD_MIN(10),
    .USE_MIN(1),
    .USE_MAX(0)
    )u_range_lux(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.lux),
    .out_in_range(out_in_range_lux)
);
   
    jump_check #(
    .WIDTH(19),
    .THRESHOLD(10000)
)u_jump_lux(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.lux),
    .temp_except(1'b0),
    .jump_error(jump_error_lux)
    );
   
    
     noise_check #(
    .WIDTH(19),
    .HISTORY(10)
)u_noise_lux(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_lux),
    .new_sample(valid),
    .noise_high(noise_high_lux)
    );
    
 //humidity

    range_check_signed #(
    .WIDTH (8),
    .THRESHOLD_MAX(100),
    .THRESHOLD_MIN(0),
    .USE_MIN(0),
    .USE_MAX(1)
    )u_range_humidity(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.humidity),
    .out_in_range(out_in_range_hum)
);
   
    jump_check #(
    .WIDTH(8),
    .THRESHOLD(10)
)u_jump_humidity(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.humidity),
    .temp_except(1'b0),
    .jump_error(jump_error_hum)
    );
    
     noise_check #(
    .WIDTH(8),
    .HISTORY(10)
)u_noise_humidity(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_hum),
    .new_sample(valid),
    .noise_high(noise_high_hum)
    );
    
//temperature
//날씨 jumpe check 조건 따로 구현
  
  logic [1:0] prev_weather;
  logic weather_change;
  logic weather_change_next; //날씨가 변한 그 다음까지 예외처리해야되니까.
  logic temp_except;
  
  always_ff @(posedge clk) begin //로직 따로 만들음
    if(!rst_n)
        prev_weather <= 2'b0;
    else if (valid)
        prev_weather <= sim_data_in.weather; //weather 여기로 들어오는지 확인
    end
    
    assign weather_change = (sim_data_in.weather != prev_weather);
    
    always_ff @(posedge clk) begin
        if(!rst_n)
         weather_change_next <= 1'b0;
    else if (valid)
        weather_change_next <= weather_change;
    end
    
    assign temp_except = weather_change || weather_change_next;

    range_check_signed #(
    .WIDTH (11),
    .THRESHOLD_MAX(220),
    .THRESHOLD_MIN(-30),
    .USE_MIN(1),
    .USE_MAX(1)
    )u_range_temperature(
    .clk(clk),
    .rst_n(rst_n),
    .value(sensor_data_in.temperature),
    .out_in_range(out_in_range_temp)
);
   
    jump_check #(    
    .WIDTH(11),
    .THRESHOLD(50)
)u_jump_temperature(
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.temperature),
    .temp_except(temp_except),
    .jump_error(jump_error_temp)
    );
    
     noise_check #(
    .WIDTH(11),
    .HISTORY(10)
)u_noise_temperature(
    .clk(clk),
    .rst_n(rst_n),
    .jump_error(jump_error_temp),
    .new_sample(valid),
    .noise_high(noise_high_temp)
    );

//온도 경고
    temp_checker u_temp_checker( 
    .clk(clk),
    .rst_n(rst_n),
    .temperature(sensor_data_in.temperature),
    .temperature_warn(temperature_warn)
    );
    

//time-out_check
    logic is_time_out;

    timeout_check #(
    .UPDATE_PERIOD(10),
    .COUNTERWIDTH(10)
    ) u_timeout_check (
    .clk(clk),
    .rst_n(rst_n),
    .valid(valid),
    .is_time_out(is_time_out)
    );
    
    assign is_timeout_distance  = is_time_out;
    assign is_timeout_approach_speed  = is_time_out;
    assign is_timeout_accel_x   = is_time_out;
    assign is_timeout_accel_y   = is_time_out;
    assign is_timeout_accel_z   = is_time_out;
    assign is_timeout_gyro_x    = is_time_out;
    assign is_timeout_gyro_y    = is_time_out;
    assign is_timeout_gyro_z    = is_time_out;
    assign is_timeout_lux       = is_time_out;
    assign is_timeout_temp      = is_time_out;
    assign is_timeout_hum       = is_time_out;
    
endmodule


