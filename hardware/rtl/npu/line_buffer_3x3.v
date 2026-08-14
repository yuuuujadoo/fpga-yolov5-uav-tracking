`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: line_buffer_3x3
// Description:
//   - 3x3 필터를 위해 픽셀 스트림을 3줄(Row)로 정렬하는 버퍼
//   - [BRAM 최적화] Vivado BRAM 추론(Inference)을 100% 보장하는 동기식 템플릿 적용
//   - [타이밍 정렬] 모든 출력(row0, row1, row2)이 정확히 1클럭 지연으로 완벽히 동기화됨
//////////////////////////////////////////////////////////////////////////////////

module line_buffer_3x3 #(
    parameter DATA_WIDTH = 8,       // 픽셀당 데이터 비트 수 (INT8)
    parameter MAX_CH = 256,         // 시스템이 지원하는 최대 채널 수
    parameter MAX_WIDTH = 320       // BRAM의 가로 최대 지원 길이
)(
    input wire clk, rst_n,          // 시스템 클럭 및 비동기 리셋
    
    input wire [10:0] img_width,    // 현재 입력 이미지의 가로 크기
    input wire [DATA_WIDTH*MAX_CH-1:0] pixel_in, // 2048-bit 최신 픽셀 스트림 버스
    input wire valid_in,            // 픽셀 입력 유효 신호
    input wire ready_in,            // 다운스트림 모듈의 Stall(멈춤) 요구 신호
    
    output reg [DATA_WIDTH*MAX_CH-1:0] row0_out, // 2줄 지연된 가장 오래된 데이터 (Top)
    output reg [DATA_WIDTH*MAX_CH-1:0] row1_out, // 1줄 지연된 중간 데이터 (Center)
    output reg [DATA_WIDTH*MAX_CH-1:0] row2_out, // 지연 없는 최신 데이터 (Bottom)
    output reg valid_out,           // 출력 픽셀 버스 유효 신호
    
    // 💡 [수술] 앞단(Top FSM)으로 현재 바쁘지 않음을 알리기 위한 배압 신호 우회
    output wire ready_out 
);

    // 단순 배선: 라인버퍼는 자체 정지 기능이 없으므로 뒷단의 상태를 앞단으로 그대로 패스
    assign ready_out = ready_in;

    // Vivado 합성기에서 블록 램(BRAM)으로 강제 추론되도록 하는 속성 선언
    (* ram_style = "block" *) reg [DATA_WIDTH*MAX_CH-1:0] ram0 [0:MAX_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH*MAX_CH-1:0] ram1 [0:MAX_WIDTH-1];

    reg [10:0] col_ptr;      // BRAM에 기록할 현재 X 좌표(포인터)
    reg [10:0] col_ptr_d1;   // Cascade 용도로 쓰일 1클럭 지연된 포인터
    reg valid_in_d1;         // Cascade 용도로 쓰일 1클럭 지연된 Valid 신호

    // 동기식 순차 회로 (파이프라인 제어)
    always @(posedge clk) begin
        if (!rst_n) begin
            // 리셋 시 제어 레지스터 초기화 (메모리 내부는 초기화 불필요)
            valid_out <= 0; valid_in_d1 <= 0;
            col_ptr <= 0; col_ptr_d1 <= 0;
            row0_out <= 0; row1_out <= 0; row2_out <= 0;
        end else if (ready_in) begin
            // Stall 상태가 아닐 때 1클럭씩 데이터 이동
            valid_in_d1 <= valid_in;
            col_ptr_d1 <= col_ptr;
            
            // 💡 [핵심 수술] 데이터(row2_out)가 BRAM 읽기 시점과 맞추어 1클럭 지연되므로,
            // valid_out 역시 valid_in을 직접 래치하여 완벽한 1클럭 지연을 맞춥니다.
            valid_out <= valid_in; 

            if (valid_in) begin
                // BRAM에서 데이터를 꺼내어 출력 레지스터에 래치 (1클럭 소요)
                row1_out <= ram0[col_ptr]; 
                row0_out <= ram1[col_ptr]; 
                row2_out <= pixel_in;      
                
                // 새로운 픽셀을 가장 윗단 버퍼(ram0)에 기록
                ram0[col_ptr] <= pixel_in; 
                
                // 가로 한 줄을 다 채우면 주소를 0으로 돌립니다 (랩어라운드)
                col_ptr <= (col_ptr == img_width - 1) ? 0 : col_ptr + 1;
            end
            
            // Cascade 폭포수 이동: 1클럭 전에 ram0에서 꺼낸 데이터를 ram1로 이동
            if (valid_in_d1) begin
                ram1[col_ptr_d1] <= row1_out;
            end
        end
    end
endmodule