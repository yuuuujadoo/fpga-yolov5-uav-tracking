`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: silu_lut_unit
// Description: 
//   - 8-bit 주소를 입력받아 사전에 계산된 SiLU 결과(8-bit)를 꺼내오는 모듈
//   - [8-bit 다이어트 적용 완료] 메모리 깊이(Depth)가 65536에서 256으로 축소됨.
//     (전체 NPU의 BRAM 사용량을 16Mbit에서 0.03Mbit 수준으로 소멸시킴)
//   - 2-Stage Pipeline: 주소 래치 -> 메모리 데이터 읽기 (ready_in Freeze 지원)
//////////////////////////////////////////////////////////////////////////////////

module silu_lut_unit #(
    parameter DATA_WIDTH = 8 // 입출력 픽셀 비트 수
)(
    input  wire clk, rst_n,
    
    input  wire signed [DATA_WIDTH-1:0] din, // 양자화 모듈에서 넘어온 8비트 주소
    input  wire valid_in,                    // 데이터 유효 깃발
    input  wire ready_in,                    // 💡 [추가] 뒷단의 멈춤 신호
    
    output wire ready_out,                   // 💡 [추가] 앞단의 멈춤 신호 (직결)
    output reg  signed [DATA_WIDTH-1:0] dout,// 활성화 함수 결과 데이터
    output reg  valid_out                    // 데이터 유효 깃발 (2클럭 지연)
);

    assign ready_out = ready_in;

    // =========================================================================
    // [1] 분산 램(Distributed RAM) 선언
    // 설명: 파이썬에서 계산된 256개의 8비트 값을 메모리에 초기화합니다.
    // =========================================================================
    reg [DATA_WIDTH-1:0] rom [0:255];
    
    initial begin
        $readmemh("silu_q6_10_int8.mem", rom);
    end

    // =========================================================================
    // [2] 파이프라인 제어 (2-Stage: 주소 래치 -> 메모리 래치)
    // =========================================================================
    reg [DATA_WIDTH-1:0] pipe_addr; // 읽어올 주소를 담는 레지스터
    reg pipe_valid;                 // 주소와 함께 이동할 1클럭 지연 유효 깃발

    // 입력된 부호 있는(Signed) 데이터를 메모리 인덱스용 부호 없는(Unsigned) 주소로 취급
    wire [7:0] rom_addr = pipe_addr;

    always @(posedge clk) begin
        if (!rst_n) begin
            dout <= 0; valid_out <= 0;
            pipe_addr <= 0; pipe_valid <= 0;
        end 
        else if (ready_in) begin // 💡 [Stall 방어] 멈추면 파이프라인 동결!
            // Stage 1: 앞단 데이터 주소 래치
            pipe_addr  <= din; 
            pipe_valid <= valid_in;
            
            // Stage 2: 메모리 데이터 읽기
            if (pipe_valid) begin
                dout <= rom[rom_addr];
            end
            valid_out <= pipe_valid; // 💡 [위상 동기화] 데이터와 동일하게 2클럭 지연
        end
    end
endmodule