`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: convolution_1x1
// Description: 
//   - 커널 크기 1x1 고정, 오직 채널(Channel) 방향으로만 폴딩(Folding) 연산을 수행
//   - 3-Stage Pipeline (Multiply -> Channel Sum -> Accumulate) — latency 원본과 동일
//   - [타이밍 수정] 채널합(Stage 2)을 한 클록 내 fabric 균형 가산 트리(16->8->4->2->1)로
//     명시 구현하고 use_dsp="no"로 fabric(CARRY4) 고정. 기존 단일 for-루프 합은
//     DSP PCIN/PCOUT 캐스케이드(14단, ~17.6ns)로 매핑되어 setup 위반을 유발했음.
//   - latency 불변(3-stage 유지), 정수 가산 결합법칙으로 결과 bit-exact 동일.
//   - [병렬도] 매 클럭마다 16개의 곱셈기(DSP)만 사용 (가산은 DSP 미사용)
//////////////////////////////////////////////////////////////////////////////////

module convolution_1x1 #(
    parameter DATA_WIDTH = 8,   // 양자화된 입력 및 가중치 비트 수 (INT8)
    parameter BIAS_WIDTH = 16,  // 양자화된 편향 비트 수 (INT16)
    parameter ACC_WIDTH = 32,   // 누적기(Accumulator) 비트 수 (오버플로우 방지용)
    parameter FOLD_CH = 16      // 한 클럭에 동시 연산할 입력 채널 수 (병렬도)
)(
    input wire clk,
    input wire rst_n,
    
    // [Control Signals]
    input wire valid_in,
    input wire last_fold_in,    // 1: 현재 픽셀 위치에서의 마지막 채널 묶음 연산임
    input wire ready_in,        // 다운스트림 모듈의 Stall 제어 신호
    
    // [Data Signals]
    input wire [(DATA_WIDTH * FOLD_CH)-1:0] pixel_in,
    input wire [(DATA_WIDTH * FOLD_CH)-1:0] weight_in,
    input wire signed [BIAS_WIDTH-1:0] bias_in, // 마지막 폴딩 사이클에서만 더해질 편향
    
    // [Output Signals]
    output reg valid_out,
    output reg signed [ACC_WIDTH-1:0] pixel_out // 32-bit 최종 누적값
);
    integer c; // 채널 루프 인덱스 변수
    integer k; // 가산 트리 인덱스 변수

    // =========================================================================
    // [Data Unpacking] (조합 회로)
    // =========================================================================
    wire signed [DATA_WIDTH-1:0] act_unpacked [0:FOLD_CH-1];
    wire signed [DATA_WIDTH-1:0] wgt_unpacked [0:FOLD_CH-1];

    genvar j;
    generate
        for (j = 0; j < FOLD_CH; j = j + 1) begin : CH_UNPACK
            localparam offset = j * DATA_WIDTH;
            assign act_unpacked[j] = $signed(pixel_in[offset +: DATA_WIDTH]);
            assign wgt_unpacked[j] = $signed(weight_in[offset +: DATA_WIDTH]);
        end
    endgenerate

    // =========================================================================
    // [Pipeline Stage 1] 곱셈 (Multiplication) - 16개 DSP
    // =========================================================================
    (* use_dsp = "yes" *) reg signed [2*DATA_WIDTH-1:0] mult_reg [0:FOLD_CH-1]; 
    
    reg valid_s1, last_fold_s1;
    reg signed [BIAS_WIDTH-1:0] bias_s1;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s1 <= 0;
            last_fold_s1 <= 0;
            bias_s1 <= 0;
            for (c = 0; c < FOLD_CH; c = c + 1) begin
                mult_reg[c] <= 0;
            end
        end
        else if (ready_in) begin
            valid_s1 <= valid_in;
            last_fold_s1 <= last_fold_in;
            bias_s1 <= bias_in;
            
            if (valid_in) begin
                for (c = 0; c < FOLD_CH; c = c + 1) begin
                    mult_reg[c] <= act_unpacked[c] * wgt_unpacked[c];
                end
            end
        end
    end

    // =========================================================================
    // [Pipeline Stage 2] 채널 합산 (Channel Sum) - 한 클록, fabric 균형트리
    //   16채널 합 최대 16*128*128 = 262144 -> 19bit. 20bit(2*DW+4) 안전.
    //   16->8->4->2->1 균형 트리를 한 always 블록 내 블로킹 가산으로 명시.
    //   use_dsp="no"로 DSP 캐스케이드 대신 fabric(CARRY4) 가산 트리로 합성됨.
    //   latency는 원본과 동일하게 valid_s1 -> valid_s2 (1단).
    // =========================================================================
    (* use_dsp = "no" *) reg signed [2*DATA_WIDTH+4-1:0] ch_sum_reg; 
    
    reg valid_s2, last_fold_s2;
    reg signed [BIAS_WIDTH-1:0] bias_s2;
    reg signed [2*DATA_WIDTH+4-1:0] t8 [0:7]; // 16->8
    reg signed [2*DATA_WIDTH+4-1:0] t4 [0:3]; // 8->4
    reg signed [2*DATA_WIDTH+4-1:0] t2 [0:1]; // 4->2

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s2 <= 0;
            last_fold_s2 <= 0; 
            bias_s2 <= 0;
            ch_sum_reg <= 0;
        end 
        else if (ready_in) begin
            valid_s2 <= valid_s1;
            last_fold_s2 <= last_fold_s1;
            bias_s2 <= bias_s1;
            
            if (valid_s1) begin
                for (k = 0; k < 8; k = k + 1)               // 16 -> 8
                    t8[k] = mult_reg[2*k] + mult_reg[2*k+1];
                for (k = 0; k < 4; k = k + 1)               // 8 -> 4
                    t4[k] = t8[2*k] + t8[2*k+1];
                t2[0] = t4[0] + t4[1];                      // 4 -> 2
                t2[1] = t4[2] + t4[3];
                ch_sum_reg <= t2[0] + t2[1];                // 2 -> 1
            end
        end
    end

    // =========================================================================
    // [Pipeline Stage 3] 누적 및 출력 (Accumulate & Output) — 원본과 동일
    //   1x1 Conv이므로 공간 합산 없이 채널합(ch_sum_reg)이 곧바로 누적기로 들어감.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [ACC_WIDTH-1:0] accumulator; // 32-bit 내부 누적기

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 0;
            pixel_out <= 0;
            accumulator <= 0;
        end 
        else if (ready_in) begin
            if (valid_s2) begin
                if (last_fold_s2) begin
                    pixel_out <= accumulator + ch_sum_reg + bias_s2;
                    valid_out <= 1;  
                    accumulator <= 0;
                end 
                else begin
                    accumulator <= accumulator + ch_sum_reg;
                    valid_out <= 0;  
                end
            end 
            else begin
                valid_out <= 0;
            end
        end
    end

endmodule
