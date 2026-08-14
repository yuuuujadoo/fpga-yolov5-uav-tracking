`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: conv_1x1_array
// Description: 
//   - 1채널짜리 convolution_1x1 엔진을 T0(16)개 만큼 병렬로 복제
//   - 단일 픽셀의 16채널 데이터(pixel_in)를 16개 엔진에 똑같이 복사 (Broadcasting)
//   - 16개의 출력 채널이 각자의 32-bit Psum을 동시에 계산하여 출력함
//////////////////////////////////////////////////////////////////////////////////

module conv_1x1_array #(
    parameter T0  = 16,      // 출력 채널 병렬 연산 수
    parameter FOLD_CH = 16,  // 한 번에 연산할 입력 채널 수
    parameter DATA_WIDTH = 8,   // 픽셀/가중치 비트 수
    parameter BIAS_WIDTH = 16,  // 편향 비트 수
    parameter PSUM_WIDTH = 32   // 누적기 최종 비트 수
)(
    input wire clk,
    input wire rst_n,

    // [Input] Pixel Data: 1픽셀 * 16채널 * 8비트 = 128-bit
    // 이 입력은 16개의 모든 1x1 Conv 엔진에 똑같이 복사됨
    input wire [DATA_WIDTH*FOLD_CH-1:0] pixel_in,
    
    // [Input] Weight Data: 16출력채널 * 1픽셀 * 16채널 * 8비트 = 2,048-bit (3x3은 18,432-bit였음!)
    // 각 1x1 Conv 엔진마다 128-bit씩 잘라서 사용함
    input wire [T0*DATA_WIDTH*FOLD_CH-1:0] weight_in,
    
    // [Input] Bias Data: 16출력채널 * 16비트 = 256-bit
    input wire [T0*BIAS_WIDTH-1:0] bias_in,

    // [Control] 제어 신호
    input wire valid_in,
    input wire last_fold_in,  
    input wire ready_in, // 전체 16개 엔진을 동시에 얼어붙게 만드는 글로벌 Stall 신호

    // [Output] 16개의 1x1 Conv 엔진에서 나오는 32-bit Psum = 512-bit
    output wire [T0*PSUM_WIDTH-1:0] psum_out,
    output wire valid_out
);
    // 1개 1x1 Conv 엔진이 먹는 가중치 비트 수 = 128
    localparam WEIGHT_PER_CH = DATA_WIDTH * FOLD_CH;

    wire signed [PSUM_WIDTH-1:0] w_psum_out [0:T0-1];
    wire [T0-1:0] w_valid_out;

    genvar i;

    // -----------------------------------------------------------------
    // 1x1 Conv 엔진 병렬 전개 (T0 = 16개 복제)
    // -----------------------------------------------------------------
    generate
        for (i = 0; i < T0; i = i + 1) begin : CONV_ENGINES
            
            // 1. 2048-bit 거대한 가중치 버스에서 현재 엔진의 128-bit 가중치만 잘라냄
            wire [WEIGHT_PER_CH-1:0] current_weight;
            assign current_weight = weight_in[(i * WEIGHT_PER_CH) +: WEIGHT_PER_CH];
            
            // 2. 편향 자르기
            wire [BIAS_WIDTH-1:0] current_bias;
            assign current_bias = bias_in[(i * BIAS_WIDTH) +: BIAS_WIDTH];

            // 3. 1x1 개별 Conv 모듈 인스턴스화
            convolution_1x1 #(
                .DATA_WIDTH(DATA_WIDTH),
                .BIAS_WIDTH(BIAS_WIDTH),
                .ACC_WIDTH(PSUM_WIDTH),
                .FOLD_CH(FOLD_CH)
            ) u_conv_1x1 (
                .clk            (clk),
                .rst_n          (rst_n),
                .valid_in       (valid_in),
                .last_fold_in   (last_fold_in),
                .ready_in       (ready_in),       
                .pixel_in       (pixel_in),       // 128-bit 픽셀 데이터 공유 (Broadcasting)
                .weight_in      (current_weight), // 128-bit 고유 가중치 할당
                .bias_in        (current_bias),   
                .valid_out      (w_valid_out[i]),
                .pixel_out      (w_psum_out[i])   
            );
        end
    endgenerate

    // -----------------------------------------------------------------
    // 출력 패킹 및 제어 신호 병합
    // -----------------------------------------------------------------
    generate
        for (i = 0; i < T0; i = i + 1) begin : PACK_PSUM
            assign psum_out[(i * PSUM_WIDTH) +: PSUM_WIDTH] = w_psum_out[i];
        end
    endgenerate

    // 모든 엔진이 동일 타이밍으로 동작하므로 0번 엔진의 valid를 대표로 사용
    assign valid_out = w_valid_out[0];

endmodule