`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: window_generator_3x3
// Description: 
//   - 3줄의 라인 버퍼 출력을 받아 3x3(9개 픽셀) 거대 윈도우를 조립하는 모듈.
//   - [에지 블리딩 차단] 화면의 오른쪽 끝 픽셀이 다음 줄의 왼쪽 끝으로 침범하는
//     시프트 레지스터의 고질적 한계를 '절대 좌표 기반 Zero-Padding MUX'로 완벽 차단.
//   - [Dummy Feed 연장] Top 래퍼에서 프레임 종료 후 밀어내기용으로 주입하는 
//     1줄 분량의 가짜 픽셀(Dummy)을 수용하기 위해 Y좌표(py) 롤백 기준을 연장함.
//   - [Zero-Padding] 합성곱 연산의 수학적 중립성을 지키기 위해 패딩값을 0으로 고정.
//////////////////////////////////////////////////////////////////////////////////

module window_generator_3x3 #(
    parameter DATA_WIDTH = 8, 
    parameter MAX_CH = 256  
)(    
    input wire clk, rst_n,
    input wire [10:0] img_width, img_height, 
    input wire [1:0]  stride,     
    
    input wire [DATA_WIDTH*MAX_CH-1:0] row0_in, row1_in, row2_in,
    input wire valid_in, ready_in,
    
    output wire [DATA_WIDTH*MAX_CH*9-1:0] window_out,
    output wire valid_out, 
    output wire ready_out
);

    // 다운스트림의 Stall 신호를 업스트림으로 바이패스
    assign ready_out = ready_in;

    // 9개의 물리적 시프트 레지스터 (3줄)
    reg [DATA_WIDTH*MAX_CH-1:0] w00, w01, w02; 
    reg [DATA_WIDTH*MAX_CH-1:0] w10, w11, w12; 
    reg [DATA_WIDTH*MAX_CH-1:0] w20, w21, w22; 

    // px, py: 가장 최신 픽셀(w22)의 물리적 입력 좌표를 추적
    reg signed [12:0] px, py; 
    reg valid_in_d1; 

    always @(posedge clk) begin
        if (!rst_n) begin
            w00 <= 0; w01 <= 0; w02 <= 0;
            w10 <= 0; w11 <= 0; w12 <= 0;
            w20 <= 0; w21 <= 0; w22 <= 0;
            // 최초 입력 전 상태
            px <= -1; py <= 0; valid_in_d1 <= 0;
        end else if (ready_in) begin
            valid_in_d1 <= valid_in;
            
            if (valid_in) begin
                // 우측(2)에서 좌측(0)으로 픽셀들을 1칸씩 밉니다.
                w00 <= w01; w01 <= w02; w02 <= row0_in;
                w10 <= w11; w11 <= w12; w12 <= row1_in;
                w20 <= w21; w21 <= w22; w22 <= row2_in;

                // 물리적 픽셀 포인터 전진 (화면 끝에 닿으면 0으로 랩어라운드)
                px <= (px == img_width - 1) ? 0 : px + 1;
                if (px == img_width - 1) py <= py + 1;
            end
        end
    end

    // 수학적 비교를 위한 부호 있는(Signed) 13비트 상수 변환
    wire signed [12:0] s_width  = {2'b00, img_width};
    wire signed [12:0] s_height = {2'b00, img_height};

    // -------------------------------------------------------------------------
    // 💡 중앙 픽셀(w11)의 절대 논리 좌표(cx, cy) 산출
    // 설명: w11은 w22보다 시간상으로 '1픽셀 전', '1줄 위'에 존재합니다.
    // px가 0이 되었을 때 중심 픽셀(w11)의 X좌표는 화면 우측 끝(s_width-1)으로 롤백합니다.
    // -------------------------------------------------------------------------
    wire signed [12:0] cx = (px == 0) ? (s_width - 1) : (px - 1);
    wire signed [12:0] cy = ((px == 0) ? py - 1 : py) - 1;

    // 중심 좌표(cx, cy)를 바탕으로 9개 픽셀의 논리적 X, Y 좌표를 주변으로 펼칩니다.
    wire signed [12:0] log_x0 = cx - 1, log_x1 = cx, log_x2 = cx + 1;
    wire signed [12:0] log_y0 = cy - 1, log_y1 = cy, log_y2 = cy + 1;

    // -------------------------------------------------------------------------
    // 💡 에지 블리딩 차단 (Bounds Check)
    // 설명: 각 픽셀의 절대 좌표가 화면 내부(0~Width-1, 0~Height-1)인지 검사합니다.
    // 시프트 레지스터 안에 반대편 화면 픽셀이 들어와 있더라도 무자비하게 걸러냅니다.
    // -------------------------------------------------------------------------
    wire v00 = (log_x0 >= 0 && log_x0 < s_width && log_y0 >= 0 && log_y0 < s_height);
    wire v01 = (log_x1 >= 0 && log_x1 < s_width && log_y0 >= 0 && log_y0 < s_height);
    wire v02 = (log_x2 >= 0 && log_x2 < s_width && log_y0 >= 0 && log_y0 < s_height);
    
    wire v10 = (log_x0 >= 0 && log_x0 < s_width && log_y1 >= 0 && log_y1 < s_height);
    wire v11 = (log_x1 >= 0 && log_x1 < s_width && log_y1 >= 0 && log_y1 < s_height);
    wire v12 = (log_x2 >= 0 && log_x2 < s_width && log_y1 >= 0 && log_y1 < s_height);
    
    wire v20 = (log_x0 >= 0 && log_x0 < s_width && log_y2 >= 0 && log_y2 < s_height);
    wire v21 = (log_x1 >= 0 && log_x1 < s_width && log_y2 >= 0 && log_y2 < s_height);
    wire v22 = (log_x2 >= 0 && log_x2 < s_width && log_y2 >= 0 && log_y2 < s_height);

    // MUX: Bounds Check 결과가 참이면 원본을, 거짓(화면 밖)이면 0을 강제 출력(제로 패딩)
    assign window_out = {
        v22 ? w22 : 2048'd0, v21 ? w21 : 2048'd0, v20 ? w20 : 2048'd0,
        v12 ? w12 : 2048'd0, v11 ? w11 : 2048'd0, v10 ? w10 : 2048'd0,
        v02 ? w02 : 2048'd0, v01 ? w01 : 2048'd0, v00 ? w00 : 2048'd0
    };

    // Stride 2 필터링: 중심 좌표(log_x1, log_y1)가 둘 다 짝수(LSB==0)일 때만 참을 반환
    wire is_valid_stride = (stride == 2) ? (log_x1[0] == 1'b0 && log_y1[0] == 1'b0) : 1'b1;

    // 💡 [Early Valid 버그 해결] 라인 버퍼가 찰 때까지는 중앙 픽셀(v11)이 화면 밖이므로 0입니다.
    // v11이 화면 안으로 들어오는 완벽한 순간(21클럭)에만 출력을 시작합니다.
    assign valid_out = valid_in_d1 && v11 && is_valid_stride;

endmodule