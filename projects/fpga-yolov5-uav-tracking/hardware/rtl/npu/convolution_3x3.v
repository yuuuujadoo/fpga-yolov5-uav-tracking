`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: convolution_3x3  (전략 C: 입력채널 시분할 — 72 곱셈기)
// Description:
//   - [전략 C — 병렬도 절감] DSP 곱셈기를 144(9x16) -> 72(9x8) 로 절반.
//     · 인터페이스(pixel_in/weight_in 16채널=1152b)는 그대로 유지.
//     · 16채널 fold 를 8채널씩 2개의 "sub-fold" 로 나눠 2 cycle 에 처리.
//       sub_in=0 -> 채널 0..7,  sub_in=1 -> 채널 8..15.
//     · 채널합/공간합은 "선형"이므로 sub0(8ch)·sub1(8ch) 의 공간-채널합을
//       누적기에 각각 더하면 16채널 결과와 비트단위 동일.
//       => 누적기는 fold 당 2개의 sub-fold 를 누적. 최종 출력은 마지막 fold 의
//          sub_in=1 (eff_last = last_fold_in & sub_in) 에서 1회.
//     · 상류(channel_folder)는 conv_block 의 divide-by-2 로 각 fold 를 2 cycle hold,
//       weight ROM 주소도 fold_idx 를 따라가 2 cycle hold => window/weight 정렬 보존.
//   - [타이밍 유지] 6-Stage 균형 가산 트리(채널 8->2->1, 공간 9->3->1). 단별 조합<=2.
//     정수 덧셈 결합법칙으로 결과 bit-exact. latency 6 유지.
//   - use_dsp: 곱셈만 DSP("yes"), 가산트리는 fabric("no") -> DSP=72/엔진.
//   - [추가] ready_in(하류 stall) 로 전체 파이프라인 freeze.
//////////////////////////////////////////////////////////////////////////////////

module convolution_3x3 #(
    parameter DATA_WIDTH = 8,   // 양자화된 픽셀/가중치 비트 수 (INT8)
    parameter BIAS_WIDTH = 16,  // 양자화된 편향 비트 수 (INT16)
    parameter ACC_WIDTH = 32,   // 누적기 비트 수 (INT32)
    parameter FOLD_CH = 16      // 입력 인터페이스 채널 수(16). 내부는 HCH=8 씩 처리
)(
    input wire clk,
    input wire rst_n,

    // [입력 데이터 버스] (16채널 인터페이스 유지)
    input wire [DATA_WIDTH*FOLD_CH*9-1:0] pixel_in,   // [Pixel0_16ch, Pixel1_16ch...]
    input wire [DATA_WIDTH*FOLD_CH*9-1:0] weight_in,  // [Ch0_9px, Ch1_9px...]
    input wire signed [BIAS_WIDTH-1:0] bias_in,       // signed (부호확장 보장)

    // [흐름 제어]
    input wire valid_in,        // 입력(sub-fold) 유효
    input wire last_fold_in,    // 현재 fold 가 채널 누적의 마지막 fold 임
    input wire sub_in,          // [전략C] 현재 sub-fold 번호 (0: ch0..7, 1: ch8..15)
    input wire ready_in,        // 외부 Stall

    // [출력]
    output reg [ACC_WIDTH-1:0] pixel_out, // 32-bit Psum
    output reg valid_out
);
    localparam HCH = FOLD_CH/2; // 내부 1 cycle 처리 채널 수 = 8

    // =========================================================================
    // [제어 신호 파이프라인] 6단: S1 -> S2a -> S2b -> S3a -> S3b -> S4
    //   eff_last = last_fold_in & sub_in  (마지막 fold 의 두번째 sub-fold 에서만 출력)
    // =========================================================================
    reg valid_s1, valid_s2a, valid_s2b, valid_s3a, valid_s3b;
    reg efl_s1, efl_s2a, efl_s2b, efl_s3a, efl_s3b; // eff_last 파이프
    reg signed [BIAS_WIDTH-1:0] bias_s1, bias_s2a, bias_s2b, bias_s3a, bias_s3b;

    integer p, c, k;

    // =========================================================================
    // [Stage 1] 곱셈 72개 (9위치 x 8채널). sub_in 으로 채널 묶음 선택.
    // =========================================================================
    (* use_dsp = "yes" *) reg signed [15:0] mult_reg [0:8][0:HCH-1]; // 9x8 = 72 곱셈 -> DSP

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s1 <= 0; efl_s1 <= 0; bias_s1 <= 0;
            for (p=0;p<9;p=p+1) for (c=0;c<HCH;c=c+1) mult_reg[p][c] <= 0;
        end else if (ready_in) begin
            valid_s1 <= valid_in;
            efl_s1   <= last_fold_in & sub_in; // 마지막 sub-fold 표시
            bias_s1  <= bias_in;
            if (valid_in) begin
                for (p=0;p<9;p=p+1) begin
                    for (c=0;c<HCH;c=c+1) begin
                        // 채널 인덱스 = sub_in*8 + c  (16채널 입력 내 위치)
                        // 픽셀: (p*FOLD_CH + ch)  / 가중치: (ch*9 + p)
                        mult_reg[p][c] <=
                            $signed(pixel_in [(p*FOLD_CH + (sub_in?HCH:0) + c)*DATA_WIDTH +: DATA_WIDTH])
                          * $signed(weight_in[(((sub_in?HCH:0) + c)*9 + p)*DATA_WIDTH +: DATA_WIDTH]);
                    end
                end
            end
        end
    end

    // =========================================================================
    // [Stage 2a] 채널 8 -> 2 (조합 2레벨: 8->4->2). 8채널 합 최대 131072 -> 20bit 안전.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [19:0] ch_l2_reg [0:8][0:1];
    (* use_dsp = "no" *) reg signed [19:0] a_l4 [0:3];
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s2a<=0; efl_s2a<=0; bias_s2a<=0;
            for(p=0;p<9;p=p+1) begin ch_l2_reg[p][0]<=0; ch_l2_reg[p][1]<=0; end
        end else if (ready_in) begin
            valid_s2a<=valid_s1; efl_s2a<=efl_s1; bias_s2a<=bias_s1;
            if (valid_s1) begin
                for(p=0;p<9;p=p+1) begin
                    // 레벨1: 8->4
                    for(k=0;k<4;k=k+1) a_l4[k] = mult_reg[p][2*k] + mult_reg[p][2*k+1];
                    // 레벨2: 4->2
                    ch_l2_reg[p][0] <= a_l4[0] + a_l4[1];
                    ch_l2_reg[p][1] <= a_l4[2] + a_l4[3];
                end
            end
        end
    end

    // =========================================================================
    // [Stage 2b] 채널 2 -> 1 (1레벨). sub-fold 1개(8채널)의 채널합 완료.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [19:0] ch_sum_reg [0:8];
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s2b<=0; efl_s2b<=0; bias_s2b<=0;
            for(p=0;p<9;p=p+1) ch_sum_reg[p]<=0;
        end else if (ready_in) begin
            valid_s2b<=valid_s2a; efl_s2b<=efl_s2a; bias_s2b<=bias_s2a;
            if (valid_s2a) for(p=0;p<9;p=p+1) ch_sum_reg[p] <= ch_l2_reg[p][0] + ch_l2_reg[p][1];
        end
    end

    // =========================================================================
    // [Stage 3a] 공간 9 -> 3 (조합 2레벨: 9->5->3). 9*131072 -> 24bit 안전.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [23:0] sp_l3_reg [0:2];
    (* use_dsp = "no" *) reg signed [23:0] s_l5 [0:4];
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s3a<=0; efl_s3a<=0; bias_s3a<=0;
            sp_l3_reg[0]<=0; sp_l3_reg[1]<=0; sp_l3_reg[2]<=0;
        end else if (ready_in) begin
            valid_s3a<=valid_s2b; efl_s3a<=efl_s2b; bias_s3a<=bias_s2b;
            if (valid_s2b) begin
                s_l5[0]=ch_sum_reg[0]+ch_sum_reg[1];
                s_l5[1]=ch_sum_reg[2]+ch_sum_reg[3];
                s_l5[2]=ch_sum_reg[4]+ch_sum_reg[5];
                s_l5[3]=ch_sum_reg[6]+ch_sum_reg[7];
                s_l5[4]=ch_sum_reg[8];
                sp_l3_reg[0]<=s_l5[0]+s_l5[1];
                sp_l3_reg[1]<=s_l5[2]+s_l5[3];
                sp_l3_reg[2]<=s_l5[4];
            end
        end
    end

    // =========================================================================
    // [Stage 3b] 공간 3 -> 1 (조합 2레벨: 3->2->1). sub-fold 1개의 공간-채널합 완료.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [23:0] spatial_sum_reg;
    (* use_dsp = "no" *) reg signed [23:0] s_l2 [0:1];
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s3b<=0; efl_s3b<=0; bias_s3b<=0; spatial_sum_reg<=0;
        end else if (ready_in) begin
            valid_s3b<=valid_s3a; efl_s3b<=efl_s3a; bias_s3b<=bias_s3a;
            if (valid_s3a) begin
                s_l2[0]=sp_l3_reg[0]+sp_l3_reg[1];
                s_l2[1]=sp_l3_reg[2];
                spatial_sum_reg <= s_l2[0]+s_l2[1];
            end
        end
    end

    // =========================================================================
    // [Stage 4] 누적 및 출력. sub-fold 마다 누적, eff_last(=last_fold&sub=1) 에서 출력.
    // =========================================================================
    (* use_dsp = "no" *) reg signed [ACC_WIDTH-1:0] accumulator;
    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out<=0; pixel_out<=0; accumulator<=0;
        end else if (ready_in) begin
            if (valid_s3b) begin
                if (efl_s3b) begin
                    // 마지막 sub-fold: 누적 + 현재 공간합 + bias -> 출력
                    pixel_out <= accumulator + spatial_sum_reg + bias_s3b;
                    valid_out <= 1;
                    accumulator <= 0;
                end else begin
                    accumulator <= accumulator + spatial_sum_reg; // sub-fold 누적
                    valid_out <= 0;
                end
            end else begin
                valid_out <= 0;
            end
        end
    end

endmodule
