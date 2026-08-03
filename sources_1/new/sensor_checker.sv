//sensor_checker- 주석 + 수정본
//abs 안 받고 안에서 계산하게 설계하


`timescale 1ns / 1ps



//////////////////////////////////////////////////////////////////////////////////
//signed_check_module
//signed, unsinged 둘다 별로 상관 없을 듯
//WIDTH - sensor data 폭, THRESHOLD는 각각 range check 시 범위, USE_MIN, USE_MAX는 min, max 중 안 쓰는게 있으면 0으로 지정하여 기능을 끄도록 설계
//clk, rst_n 은 필요 없을 수 있는데 일단 모든 모듈의 통일성을 위해 넣어둠, value는 받는 센서 측정값에 해당
module range_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD_MAX = 100,
    parameter THRESHOLD_MIN = 100,
    parameter USE_MIN = 1'b1,
    parameter USE_MAX = 1'b1
)( 
    input logic signed [WIDTH-1:0] sensor_data, // 수정: signed
    output logic range_error
    );
//기준 이상 밑 이하의 값이 나올 경우 out_in_range, USE_MIN,USE_MAX와 AND 처리 되어있어서 0이 될시 자동적으로 if 문 안이 0이됨
    always_comb begin
    range_error = 1'b0;

    if (USE_MIN && (sensor_data < THRESHOLD_MIN))
        range_error = 1'b1;

    if (USE_MAX && (sensor_data > THRESHOLD_MAX))
        range_error = 1'b1;
    end
endmodule


// jump check 의 경우에는 과거와 현재 값을 둘 다 사용할 수 있어야한다. ->즉 register가 필요하기 때문에 다른 모듈과의 timing에 대해서는 생각해보는것도 나쁘지 않을지도

module jump_check #(
    parameter WIDTH = 12,
    parameter THRESHOLD = 100
)(
    input logic signed [WIDTH-1:0] delta_tendency,
    input logic distance_except,
    input logic weather_change,
    output logic jump_error
    );
///////////////////////////////////////////////////////////  
    //절댓값이 기준값보다 작으면 jump_error=0. temp_except = 1(날씨 전환되는 시점)이면 무조건 0
    always_comb begin 
        if (weather_change || distance_except)
           jump_error = 1'b0;
        else if (delta_tendency >= -THRESHOLD && delta_tendency <= THRESHOLD) 
	       jump_error = 1'b0; 
        else jump_error = 1'b1; 
    end 
endmodule

//stuck check
//사실 HISTORY는 뺴도 되긴하다 만약 HISTORY 길이를 전부 10비트로 맞출거라면
//new_sample 받아와서 신호가 들어왔음을 인지해야한다
module stuck_check #(
    parameter WIDTH=16,
    parameter HISTORY =10,
    parameter THRESHOLD=10
)(
    input logic clk,
    input logic rst_n,
    input logic new_sample,
    input logic signed [WIDTH-1:0] sensor_data,
    input logic check_enable,
    output logic stuck_error
    );
    logic [HISTORY-1:0] stuck_history;
    logic [$clog2(HISTORY+1)-1:0] stuck_count;
    
    //우선 reset, 그후 new sample이 들어올때 stuck history 는 left shift 하면서, (diff < THRESHOLD)&&check_enable가 true면 1, 아니면 0을 LSB에 채운다. 그후 prev_data를 업데이트
    //즉 stuck_history의 경우 기존 10개의 값에 대해 check를 진행하게 됨, 그중 특정 개수만큼이 1이면 stuck 으로 판단
    //이때 check_enable의 경우에는 조건식이 온다. 예를 들어 distance != 20000이면 distance 이 20000인 경우에는 자동적으로 0이되게 하던가 혹은 weather delta !=0 이면 날씨가 바뀔 때 1이 되게하는 등 
    //조건을 만족하게
    always_ff @(posedge clk) begin //수정: 동기 리셋
        if (!rst_n) begin 
        prev_data <= '0; 
        stuck_history <= '0; 
    end 
    else if (new_sample) begin 
    stuck_history <= { stuck_history[HISTORY-2:0], (diff < THRESHOLD)&&check_enable}; 
    prev_data <= current_data; 
    end 
    end 

    //stuck_count의 경우 최근 HISTORY만큼의 데이터 중 몇개가 stuck으로 count되었는지 확인하는것
    always_comb begin 
	stuck_count = 0; 
        for (int i = 0; i < HISTORY; i++) begin 
        stuck_count = stuck_count + stuck_history[i]; 
        end 
    end 
    //최종적으로 is_stuck 판단 여부 (stuck 8회 이상 되었을 때 is_stuck가 됨)
    always_comb begin 
    is_stuck = (stuck_count >= 8); 
    end 
endmodule

//time_out check
//UPDATE_PERIOD는 따로 받아와야한다(업데이트 주기값을 알아서 와야함)
//COUNTERWIDTH의 경우 history와 유사한 역할
module timeout_check #(
    parameter UPDATE_PERIOD = 10, // 나중에 결정될 값
    parameter COUNTERWIDTH =10
    )(
    input logic clk,
    input logic rst_n,
    input  logic valid,
    output logic is_time_out
    );
    logic [COUNTERWIDTH-1:0] count;
//1클락 마다 counter가 증가하는데 valid 들어오면 0으로 리셋. 즉 만약 clock 이 업데이트주기의 2배 이상 돌아 count 값이 커졌는데 리셋이 안됨 --> valid 값이 입력되지 않고 있음
always_ff @(posedge clk) begin // 수정: 동기리셋
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
//실제 current 값을 받을 필요 없고 마찬가지로 history만 필요함-->new_sample만 받으면 될듯
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
    
    //마찬가지로 newsample 받아오면 LSB에 jump_error 값이 들어옴
    //이때 jump_error 는 위의 jump_error 모듈에서 logic으로 빼와서 받아와야함
    always_ff @(posedge clk) begin // 수정: 동기리셋
    if (!rst_n) begin
       noise_history <= '0;
      end
    else if (new_sample) begin
        noise_history <= {
            noise_history[HISTORY-2:0],jump_error};
    end
    end
//아까 stuck check 의 논리와 유사함    
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
   
// 여기서부터는 temperature 특정 범위에 따라 warning 뜨게 하는 모듈에 해당한다
 //temperature_warn
module temp_checker(
    input logic clk,
    input logic rst_n,
    input logic signed [10:0] temperature,

    output logic temperature_warn
    );
    
always_comb begin
    temperature_warn = (temperature <= -200) || (temperature >= 500); // 추가
end

endmodule

//timing 여부가 맞는지는 확인하지 못함
//temp, vlotage의 경우 구체적인 수치는 다시 바꿔놔야함 특히 11.2 처럼 소수점 어떻게 할지
//속도 기준은 아직 짜지 못함
//온도의 jumpcheck의 경우에는 불러온 모듈 밖에서 코드를 하나 더 만들어야할듯
//sensor_reliability output에 뭐 연결할지 못정함
//input에 abs 지우지 못함
//속도 모듈 아직 없음
//아마 값이나 이런거 틀린거 있울 수 있고, 모듈 input output에 연결한 변수명, 모듈 불러온 인스턴스명 잘 확인해야할듯(제가 복붙 시킨거라 변수명이 안바뀌었을수도 있습니다..)
