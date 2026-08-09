`timescale 1ns / 1ps

//output package pkg���� ��������ҵ�, output port �����������
//current value�� [WIDHT-1:0]���� ���� ��
//delta�� abs�� �־���� ���� ����
//noise check �� �� jump_error �� �ʿ� --> logic���� ���� ����������� 
//new_sample ��� �޴���... �׸��� sensor input�� preprocessed�� ���� ���� �� �ִ� timing�� �Ǵ����� Ȯ��������Ԥ�
//stuck check ��ȭ�� ���� ���� ������ �ʿ��� �� ���� ���� input delta�� �޾ƿͼ� if delta>0 �� ���� �������� �ٲٸ� �ɵ�

//����: ����, �µ�, ����, ���п� ���ؼ��� stuck check ���� ����. �����Ȱ� ������ ���̱� ����

import types_pkg::*;

module top_sensor_checker(
    input logic clk,
    input logic rst_n,
    input logic valid,
    input sensor_data_t sensor_data_in,
    input sim_data_t    sim_data_in,
    output logic out_in_range_distance, jump_error_distance, is_stuck_distance, noise_high_distance, // output �߰�
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
//�Ÿ� stuck check �ָ��� 

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
    .temp_except(1'b0), // ����: abs ��� temp_except
    .jump_error(jump_error_distance)
    );
    
    // �Ÿ� Stuck check ���� �����
    logic [7:0] prev_speed;
    
    always_ff @(posedge clk) begin
        if(!rst_n)
            prev_speed <= 8'd0;
        else if (valid)
            prev_speed <= Speed;
    end
    
    logic speed_change; //���� �ӵ� ��ȭ�ϴ���
    assign speed_change = (Speed != prev_speed); //sensor_data_in�� Speed �ִ��� Ȯ�� �ʿ�
    
    logic distance_stuck_check_en;
    assign distance_stuck_check_en = (sensor_data_in.distance != 16'd20000) && speed_change;
    
    
    
    stuck_check #(
    .WIDTH(16),
    .HISTORY (10),
    .THRESHOLD(1)
)u_stuck_check_distance(  //����: �Ÿ� �ŷڵ� �Ǵܿ� ���� ���� ����
    .clk(clk),
    .rst_n(rst_n),
    .current_data(sensor_data_in.distance),
    .new_sample(valid),
    .check_enable(distance_stuck_check_en),
    .is_stuck(is_stuck_distance) // ����: abs ����
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

// ���ټӵ� reliability. �ǹ�: Ȯ���ʿ�
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
        .THRESHOLD(200) //�ùķ��̼� ���� Ȯ�� �ʿ�
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
        .current_data(sensor_data_in.approach_speed), // �� ��ȣ �� ����� ���پ�����.
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
    
    always_ff @(posedge clk) begin           //���� �߰�
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
    logic accel_z_stuck_check_en; // ���� ������ֱ�
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
    
    stuck_check #( //�ӵ��� 0�� �ƴҶ��� ��� �ݿ���.
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
//���� jumpe check ���� ���� ����
  
  logic [1:0] prev_weather;
  logic weather_change;
  logic weather_change_next; //������ ���� �� �������� ����ó���ؾߵǴϱ�.
  logic temp_except;
  
  always_ff @(posedge clk) begin //���� ���� ������
    if(!rst_n)
        prev_weather <= 2'b0;
    else if (valid)
        prev_weather <= sim_data_in.weather; //weather ����� �������� Ȯ��
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

//�µ� ���
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


