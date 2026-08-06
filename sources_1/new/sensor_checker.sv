//sensor_checker- �ּ� + ������
//abs �� �ް� �ȿ��� ����ϰ� ������


`timescale 1ns / 1ps

import types_pkg::*;



//////////////////////////////////////////////////////////////////////////////////
//signed_check_module
//signed, unsinged �Ѵ� ���� ��� ���� ��
//WIDTH - sensor data ��, THRESHOLD�� ���� range check �� ����, USE_MIN, USE_MAX�� min, max �� �� ���°� ������ 0���� �����Ͽ� ����� ������ ����
//clk, rst_n �� �ʿ� ���� �� �ִµ� �ϴ� ��� ����� ���ϼ��� ���� �־��, value�� �޴� ���� �������� �ش�
module range_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD_MAX = 100,
    parameter THRESHOLD_MIN = 100,
    parameter USE_MIN = 1'b1,
    parameter USE_MAX = 1'b1
)( 
    input logic signed [WIDTH-1:0] sensor_data, // ����: signed
    output logic range_error
    );
//���� �̻� �� ������ ���� ���� ��� out_in_range, USE_MIN,USE_MAX�� AND ó�� �Ǿ��־ 0�� �ɽ� �ڵ������� if �� ���� 0�̵�
    always_comb begin
    range_error = 1'b0;

    if (USE_MIN && (sensor_data < THRESHOLD_MIN))
        range_error = 1'b1;

    if (USE_MAX && (sensor_data > THRESHOLD_MAX))
        range_error = 1'b1;
    end
endmodule


// jump check �� ��쿡�� ���ſ� ���� ���� �� �� ����� �� �־���Ѵ�. ->�� register�� �ʿ��ϱ� ������ �ٸ� ������ timing�� ���ؼ��� �����غ��°͵� ������ ��������

module jump_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD = 100
)(
    input logic signed [WIDTH-1:0],
    input logic weather_change,
    output logic jump_error
    );
///////////////////////////////////////////////////////////  
    //������ ���ذ����� ������ jump_error=0. temp_except = 1(���� ��ȯ�Ǵ� ����)�̸� ������ 0
    always_comb begin 
        if (weather_change || distance_except)
           jump_error = 1'b0;
        else if ( >= -THRESHOLD && delta_tendency <= THRESHOLD) 
	       jump_error = 1'b0; 
        else jump_error = 1'b1; 
    end 
endmodule

//stuck check

module stuck_check #(
    parameter int          WIDTH       = 16,
    parameter int          THRESHOLD   = 0,    // ← signed 유지
    parameter int          CHANNEL_NUM = 1,
    parameter int          TW          = 20,
    parameter int unsigned U           = 1,    // ← unsigned
    parameter int unsigned D           = 1,
    parameter int unsigned N           = 10
)(
    input logic clk,
    input logic rst_n,
    input logic valid_s1,
    input logic signed [WIDTH-1:0] sensor_data,
    input logic signed [WIDTH-1:0] prev_sensor_data,
    input logic signed [TW-1:0] trig_val,
    output logic stuck_error
    );
    
    logic raw_stuck;
    logic cond_b;
    logic [$clog2(N+1)-1:0] stuck_cnt;

    always_comb begin
        case (CHANNEL_NUM)
            1:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 거리     <- Sum(접근속도)
            2:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 접근속도 <- 거리 residual
            3:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 속도x    <- Sum(가속도x)
            4:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 속도y    <- Sum(가속도y)
            5:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 속도z    <- Sum(가속도z)
            6:  cond_b = (trig_val != '0);                                      // 가속도x  <- delta 속도x
            7:  cond_b = (trig_val != '0);                                      // 가속도y  <- delta 속도y
            8:  cond_b = (trig_val != '0);                                      // 가속도z  <- delta 속도z
            9:  cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 기울기x  <- Sum(각속도x)
            10: cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 기울기y  <- Sum(각속도y)
            11: cond_b = (trig_val >  THRESHOLD) || (trig_val < -THRESHOLD);    // 기울기z  <- Sum(각속도z)
            12: cond_b = (trig_val != '0);                                      // 각속도x  <- delta 기울기x
            13: cond_b = (trig_val != '0);                                      // 각속도y  <- delta 기울기y
            14: cond_b = (trig_val != '0);                                      // 각속도z  <- delta 방향
            15: cond_b = 1'b1;                                                  // 온도     <- 폴백 (트리거 없음)
            16: cond_b = 1'b1;                                                  // 습도     <- 폴백 (트리거 없음)
            17: cond_b = 1'b1;                                                  // 조도     <- 폴백 (트리거 없음)
            default: cond_b = 1'b0;
        endcase

        if (valid_s1) raw_stuck = ((sensor_data == prev_sensor_data) && cond_b) ? 1'b1 : 1'b0;
        else raw_stuck = 1'b0;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            stuck_cnt   <= '0;
            stuck_error <= 1'b0;
        end
        else begin
            if (valid_s1) begin 
                stuck_cnt <= (raw_stuck == 1'b1) ? ((stuck_cnt + U > N) ? N : N+U) : ((stuck_cnt < D) ? '0 : stuck_cnt - D);
                stuck_error <= (stuck_cnt == N) ? 1'b1 : 1'b0;
            end
        end
    end
endmodule

//time_out check
//UPDATE_PERIOD�� ���� �޾ƿ;��Ѵ�(������Ʈ �ֱⰪ�� �˾Ƽ� �;���)
//COUNTERWIDTH�� ��� history�� ������ ����
module timeout_check #(
    parameter UPDATE_PERIOD = 10, // ���߿� ������ ��
    parameter COUNTERWIDTH =10
    )(
    input logic clk,
    input logic rst_n,
    input  logic valid,
    output logic is_time_out
    );
    logic [COUNTERWIDTH-1:0] count;
//1Ŭ�� ���� counter�� �����ϴµ� valid ������ 0���� ����. �� ���� clock �� ������Ʈ�ֱ��� 2�� �̻� ���� count ���� Ŀ���µ� ������ �ȵ� --> valid ���� �Էµ��� �ʰ� ����
always_ff @(posedge clk) begin // ����: ���⸮��
    if (!rst_n  )
        count <= 0;
    else if(valid)
        count <=0;
   else
        count <= count + 1;
end
    always_comb begin
    is_time_out = (count >= 2*UPDATE_PERIOD);
    end
endmodule


//noise_check
//���� current ���� ���� �ʿ� ���� ���������� history�� �ʿ���-->new_sample�� ������ �ɵ�
module noise_check #(
    parameter WIDTH =16,
    parameter HISTORY =10
)(
    input logic clk,
    input logic rst_n,
    input logic jump_error,
    input logic new_sample,
    output logic noise_high
    );
    logic [HISTORY-1:0] noise_history;
    logic [$clog2(HISTORY+1)-1:0] noise_count;
    
    //���������� newsample �޾ƿ��� LSB�� jump_error ���� ����
    //�̶� jump_error �� ���� jump_error ��⿡�� logic���� ���ͼ� �޾ƿ;���
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
   
// ���⼭���ʹ� temperature Ư�� ������ ���� warning �߰� �ϴ� ��⿡ �ش��Ѵ�
 //temperature_warn
module temp_checker(
    input logic clk,
    input logic rst_n,
    input logic signed [10:0] temperature,

    output logic temperature_warn
    );
    
always_comb begin
    temperature_warn = (temperature <= -200) || (temperature >= 500); // �߰�
end

endmodule

//timing ���ΰ� �´����� Ȯ������ ����
//temp, vlotage�� ��� ��ü���� ��ġ�� �ٽ� �ٲ������ Ư�� 11.2 ó�� �Ҽ��� ��� ����
//�ӵ� ������ ���� ¥�� ����
//�µ��� jumpcheck�� ��쿡�� �ҷ��� ��� �ۿ��� �ڵ带 �ϳ� �� �������ҵ�
//sensor_reliability output�� �� �������� ������
//input�� abs ������ ����
//�ӵ� ��� ���� ����
//�Ƹ� ���̳� �̷��� Ʋ���� �ֿ� �� �ְ�, ��� input output�� ������ ������, ��� �ҷ��� �ν��Ͻ��� �� Ȯ���ؾ��ҵ�(���� ���� ��Ų�Ŷ� �������� �ȹٲ�������� �ֽ��ϴ�..)
