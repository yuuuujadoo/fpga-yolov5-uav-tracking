`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: post_processor
// Description: 
//   - 32-bit Psum 배열을 입력받아 Requantization(클리핑)과 SiLU(또는 Bypass)를
//     수행하는 "순수 범용 후처리 코어"
//
//   [아키텍처 수술 내역: Control Signal Pipelining]
//   - 데이터는 5-Clock(Requant 3 + SiLU 2) 뒤에 나오는데, 제어 신호(silu_en)가
//     지연 없이 즉시 MUX를 스위칭하여 데이터가 뒤섞이는 치명적 버그를 해결했습니다.
//   - 5-Stage 시프트 레지스터(silu_en_pipe)를 도입하여 데이터의 흐름과 
//     스위치 신호의 흐름을 100% 동일한 레이턴시로 묶어냈습니다. (Freeze 완벽 지원)
//////////////////////////////////////////////////////////////////////////////////

module post_processor #(
    parameter T0         = 16,  
    parameter PSUM_WIDTH = 32,  
    parameter M0_WIDTH   = 16,  
    parameter REQ_WIDTH  = 8,   
    parameter OUT_WIDTH  = 8    
)(
    input wire clk, rst_n,

    // [Dynamic Configuration] 
    input wire [T0*M0_WIDTH-1:0] M0_in,   
    input wire [T0*5-1:0]        n_in,    
    input wire                   silu_en, // 💡 Head 레이어에서 SiLU를 끄기 위한 스위치

    // [Data/Control In]
    input wire [T0*PSUM_WIDTH-1:0] psum_in,
    input wire valid_in,
    input wire ready_in,   // 💡 [추가] 메모리(DMA)가 꽉 찼을 때 보내는 브레이크
    
    // [Data/Control Out]
    output wire ready_out, // 💡 [추가] 앞단(Conv Array)을 세우는 브레이크
    output wire [T0*OUT_WIDTH-1:0] pixel_out,
    output wire valid_out
);

    // =========================================================================
    // [1] 재양자화 블록 (3-Clock Latency)
    // =========================================================================
    wire [T0*REQ_WIDTH-1:0] requant_out;
    wire requant_valid;
    wire silu_ready_out;

    requant_array #(
        .T0(T0), .PSUM_WIDTH(PSUM_WIDTH), .M0_WIDTH(M0_WIDTH), .OUT_WIDTH(REQ_WIDTH)
    ) u_requant (
        .clk(clk), .rst_n(rst_n),
        .psum_in(psum_in), .M0_in(M0_in), .n_in(n_in),
        .valid_in(valid_in),
        .ready_in(silu_ready_out), // SiLU 모듈의 준비 상태를 받음
        .ready_out(ready_out),     // MAC 어레이로 상태 전달
        .requant_out(requant_out), .valid_out(requant_valid)
    );

    // =========================================================================
    // [2] SiLU 활성화 블록 (2-Clock Latency)
    // =========================================================================
    wire [T0*OUT_WIDTH-1:0] silu_out;
    wire silu_valid;

    activation_silu #(
        .T0(T0), .DATA_WIDTH(OUT_WIDTH)  
    ) u_silu_array (
        .clk(clk), .rst_n(rst_n),
        .requant_data_in(requant_out), .valid_in(requant_valid),
        .ready_in(ready_in), // DMA의 멈춤 상태 수신
        .ready_out(silu_ready_out),
        .silu_data_out(silu_out), .valid_out(silu_valid)
    );

    // =========================================================================
    // [3] 💡 Bypass 파이프라인 밸런싱 (SiLU를 안 거치는 데이터의 시간 끌기)
    // 설명: SiLU가 연산하는 2클럭 동안, Bypass되는 양자화 결과물도 허공에 띄워두고 
    // 2클럭 지연시켜야 나중에 MUX에서 만날 때 짝이 맞습니다.
    // =========================================================================
    reg [T0*OUT_WIDTH-1:0] delay1_req_out, delay2_req_out;
    reg                    delay1_req_val, delay2_req_val;

    always @(posedge clk) begin
        if (!rst_n) begin
            delay1_req_out <= 0; delay2_req_out <= 0;
            delay1_req_val <= 0; delay2_req_val <= 0;
        end
        else if (ready_in) begin // 💡 [Stall 방어] 멈추면 대기조 데이터도 얼음!
            // 1클럭 지연
            delay1_req_out <= requant_out;
            delay1_req_val <= requant_valid;
            
            // 2클럭 지연 (SiLU 출력 타이밍과 정확히 일치됨)
            delay2_req_out <= delay1_req_out;
            delay2_req_val <= delay1_req_val;
        end
    end

    // =========================================================================
    // [4] 💡 제어 신호(silu_en) 시프트 레지스터 (5-Clock Latency Matching)
    // 설명: 맨 처음에 들어온 스위치 신호(silu_en)가 5클럭을 여행하고 나온 데이터들과
    // 만나기 위해서는, 스위치 신호 자체도 컨베이어 벨트에 태워 5클럭을 지연시켜야 합니다.
    // =========================================================================
    reg silu_en_d1, silu_en_d2, silu_en_d3, silu_en_d4, silu_en_d5;

    always @(posedge clk) begin
        if (!rst_n) begin
            silu_en_d1 <= 0; silu_en_d2 <= 0; silu_en_d3 <= 0; 
            silu_en_d4 <= 0; silu_en_d5 <= 0;
        end
        else if (ready_in) begin // 💡 [Stall 방어] 스위치 신호도 같이 동결!
            silu_en_d1 <= silu_en;
            silu_en_d2 <= silu_en_d1;
            silu_en_d3 <= silu_en_d2;
            silu_en_d4 <= silu_en_d3;
            silu_en_d5 <= silu_en_d4; // 5번째 클럭에서 데이터와 상봉
        end
    end

    // =========================================================================
    // [5] 최종 출력 MUX (Bypass vs SiLU 선택)
    // =========================================================================
    // 5클럭 지연되어 도착한 제어 신호에 따라 출력을 선택합니다.
    assign pixel_out = silu_en_d5 ? silu_out   : delay2_req_out;
    assign valid_out = silu_en_d5 ? silu_valid : delay2_req_val;

endmodule