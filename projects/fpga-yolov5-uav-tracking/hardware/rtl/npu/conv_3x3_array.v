`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: conv_3x3_array
// Description: 
//   - 기존 1채널짜리 convolution_3x3 엔진을 T0(16)개 만큼 병렬로 복제
//   - 입력 특징맵(window)은 16개 엔진에 똑같이 복사 (Broadcasting)
//   - 16개의 출력 채널이 각자의 32-bit Psum을 동시에 계산하여 출력함
//   - [추가] ready_in 신호를 받아 16개의 하위 모듈에 동시에 뿌려 완벽한 병렬 Stall 제어
//////////////////////////////////////////////////////////////////////////////////

module conv_3x3_array #(
    parameter T0  = 16,      // 출력 채널 병렬 연산 수
    parameter FOLD_CH = 16,  // 한 번에 연산할 입력 채널 수
    parameter DATA_WIDTH = 8,   // 픽셀/가중치 비트 수
    parameter BIAS_WIDTH = 16,  // 편향 비트 수
    parameter PSUM_WIDTH = 32   // 누적기 최종 비트 수
)(
    input wire clk,
    input wire rst_n,

    // [Input] Window Data: 16채널 * 9픽셀 * 8비트 = 1152-bit
    // 이 입력은 16개의 모든 Conv 엔진에 똑같이 복사되어 들어감 (Broadcasting)
    input wire [DATA_WIDTH*FOLD_CH*9-1:0] window_in,
    
    // [Input] Weight Data: 16출력채널 * 16입력채널 * 9픽셀 * 8비트 = 18,432-bit
    // 각 Conv 엔진마다 자신의 가중치 영역을 잘라서 사용함
    input wire [T0*DATA_WIDTH*FOLD_CH*9-1:0] weight_in,
    
    // [Input] Bias Data: 16출력채널 * 16비트 = 256-bit
    input wire [T0*BIAS_WIDTH-1:0] bias_in,

    // [Control] 제어 신호
    input wire valid_in,
    input wire last_fold_in,  // 1이면 누적 종료 및 최종 픽셀 출력
    input wire sub_in,        // [전략C] sub-fold 번호 (0:ch0-7, 1:ch8-15) — 16엔진 공통 브로드캐스트
    
    // [핵심 추가] 뒷단 모듈(Top 또는 Requantization)에서 들어오는 Stall 신호
    input wire ready_in,

    // [Output] 16개의 Conv 엔진에서 쏟아져 나오는 32-bit Psum = 512-bit
    output wire [T0*PSUM_WIDTH-1:0] psum_out,
    output wire valid_out
);

    // 파라미터 기반 개별 데이터 폭 계산
    localparam WEIGHT_PER_CH = DATA_WIDTH * FOLD_CH * 9; // 1개 Conv 엔진이 먹는 가중치 비트 수 (1152)

    wire signed [PSUM_WIDTH-1:0] w_psum_out [0:T0-1];
    wire [T0-1:0] w_valid_out;

    genvar i;

    // -----------------------------------------------------------------
    // Conv 엔진 병렬 전개 (T0 = 16개 복제)
    // -----------------------------------------------------------------
    generate
        for (i = 0; i < T0; i = i + 1) begin : CONV_ENGINES
            
            // 1. 거대한 가중치 버스에서 현재 i번째 엔진의 가중치만 잘라냄
            wire [WEIGHT_PER_CH-1:0] current_weight;
            assign current_weight = weight_in[(i * WEIGHT_PER_CH) +: WEIGHT_PER_CH];
            
            // 2. 거대한 편향 버스에서 현재 i번째 엔진의 편향만 잘라냄
            wire [BIAS_WIDTH-1:0] current_bias;
            assign current_bias = bias_in[(i * BIAS_WIDTH) +: BIAS_WIDTH];

            // 3. 개별 Conv 모듈 인스턴스화
            convolution_3x3 #(
                .DATA_WIDTH(DATA_WIDTH),
                .BIAS_WIDTH(BIAS_WIDTH),
                .ACC_WIDTH(PSUM_WIDTH),
                .FOLD_CH(FOLD_CH)
            ) u_conv (
                .clk            (clk),
                .rst_n          (rst_n),
                .valid_in       (valid_in),
                .last_fold_in   (last_fold_in),
                .sub_in         (sub_in),         // [전략C] sub-fold 번호 브로드캐스트
                .ready_in       (ready_in),       // [추가] 정지 신호를 16개 엔진에 똑같이 분배
                .pixel_in       (window_in),      // [수정] 모든 엔진이 동일한 1152-bit 윈도우 공유
                .weight_in      (current_weight), // 잘라낸 1152-bit 고유 가중치 할당
                .bias_in        (current_bias),   // 잘라낸 고유 편향 할당
                .valid_out      (w_valid_out[i]),
                .pixel_out      (w_psum_out[i])   // 32-bit Psum 출력
            );
        end
    endgenerate

    // -----------------------------------------------------------------
    // 출력 패킹 및 제어 신호 대표값 설정
    // -----------------------------------------------------------------
    generate
        for (i = 0; i < T0; i = i + 1) begin : PACK_PSUM
            // 16개의 32-bit 출력을 512-bit 거대 버스로 일렬 패킹
            assign psum_out[(i * PSUM_WIDTH) +: PSUM_WIDTH] = w_psum_out[i];
        end
    endgenerate

    // 모든 엔진이 완벽하게 동일한 파이프라인 타이밍을 가지므로, 
    // 0번 엔진의 유효 신호 1개만 뽑아서 전체 Array의 대표 유효 신호로 사용합니다.
    assign valid_out = w_valid_out[0];

endmodule