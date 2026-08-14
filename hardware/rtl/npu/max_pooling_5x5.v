`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: max_pooling_5x5
// Description: 
//   - 단 1개의 채널에 대한 5x5(25픽셀) 윈도우를 입력받아 최댓값 1개를 출력하는 코어.
//   - [비교기 트리] DSP(곱셈기) 없이 순수 LUT 로직만으로 구성.
//   - [4-Stage Pipeline] 사용자님의 Conv 모듈과 완벽히 동일한 4단계 파이프라인 지연을
//     가지며, ready_in(Stall/Freeze) 신호를 지원하여 타이밍을 일치시킴.
//   - [Signed Data] YOLOv5의 활성화 값은 Signed 8-bit(-128~127)이므로 부호 있는 비교 수행.
//////////////////////////////////////////////////////////////////////////////////

module max_pooling_5x5 #(
    parameter DATA_WIDTH = 8 // INT8 (-128 ~ 127)
)(
    input wire clk, rst_n,
    input wire [DATA_WIDTH*25-1:0] window_in,
    input wire valid_in, last_fold_in, ready_in,
    
    output reg signed [DATA_WIDTH-1:0] max_out,
    output reg valid_out, last_fold_out
);

    // 부호 있는(Signed) 8비트 크기 비교기 함수
    function signed [DATA_WIDTH-1:0] max2;
        input signed [DATA_WIDTH-1:0] a, b;
        begin
            max2 = (a > b) ? a : b;
        end
    endfunction

    // 💡 [배열 크기 정상화] 파이프라인 단계별 필요 레지스터
    reg signed [DATA_WIDTH-1:0] s1_reg [0:12]; // 25 -> 13개로 압축
    reg signed [DATA_WIDTH-1:0] s2_reg [0:6];  // 13 -> 7개로 압축
    reg signed [DATA_WIDTH-1:0] s3_reg [0:3];  // 7 -> 4개로 압축
    
    reg valid_s1, valid_s2, valid_s3;
    reg last_fold_s1, last_fold_s2, last_fold_s3;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_s1 <= 0; valid_s2 <= 0; valid_s3 <= 0; valid_out <= 0;
            last_fold_s1 <= 0; last_fold_s2 <= 0; last_fold_s3 <= 0; last_fold_out <= 0;
            max_out <= 0;
            // 모든 내부 레지스터를 확실하게 0으로 초기화
            for(i=0; i<=12; i=i+1) s1_reg[i] <= 0;
            for(i=0; i<=6; i=i+1)  s2_reg[i] <= 0;
            for(i=0; i<=3; i=i+1)  s3_reg[i] <= 0;
        end 
        else if (ready_in) begin // 💡 [Stall 방어] 
            // 제어 깃발 패스
            valid_s1 <= valid_in; valid_s2 <= valid_s1; valid_s3 <= valid_s2; valid_out <= valid_s3;
            last_fold_s1 <= last_fold_in; last_fold_s2 <= last_fold_s1; last_fold_s3 <= last_fold_s2; last_fold_out <= last_fold_s3;
            
            // -----------------------------------------------------------------
            // Stage 1: 25개 -> 13개
            // -----------------------------------------------------------------
            if (valid_in) begin
                s1_reg[0] <= max2($signed(window_in[0*8+:8]), $signed(window_in[1*8+:8]));
                s1_reg[1] <= max2($signed(window_in[2*8+:8]), $signed(window_in[3*8+:8]));
                s1_reg[2] <= max2($signed(window_in[4*8+:8]), $signed(window_in[5*8+:8]));
                s1_reg[3] <= max2($signed(window_in[6*8+:8]), $signed(window_in[7*8+:8]));
                s1_reg[4] <= max2($signed(window_in[8*8+:8]), $signed(window_in[9*8+:8]));
                s1_reg[5] <= max2($signed(window_in[10*8+:8]), $signed(window_in[11*8+:8]));
                s1_reg[6] <= max2($signed(window_in[12*8+:8]), $signed(window_in[13*8+:8]));
                s1_reg[7] <= max2($signed(window_in[14*8+:8]), $signed(window_in[15*8+:8]));
                s1_reg[8] <= max2($signed(window_in[16*8+:8]), $signed(window_in[17*8+:8]));
                s1_reg[9] <= max2($signed(window_in[18*8+:8]), $signed(window_in[19*8+:8]));
                s1_reg[10]<= max2($signed(window_in[20*8+:8]), $signed(window_in[21*8+:8]));
                s1_reg[11]<= max2($signed(window_in[22*8+:8]), $signed(window_in[23*8+:8]));
                s1_reg[12]<= $signed(window_in[24*8+:8]); // 짝이 없는 마지막 1개는 그대로 내림
            end
            
            // -----------------------------------------------------------------
            // Stage 2: 13개 -> 7개
            // -----------------------------------------------------------------
            if (valid_s1) begin
                s2_reg[0] <= max2(s1_reg[0], s1_reg[1]);
                s2_reg[1] <= max2(s1_reg[2], s1_reg[3]);
                s2_reg[2] <= max2(s1_reg[4], s1_reg[5]);
                s2_reg[3] <= max2(s1_reg[6], s1_reg[7]);
                s2_reg[4] <= max2(s1_reg[8], s1_reg[9]);
                s2_reg[5] <= max2(s1_reg[10], s1_reg[11]);
                s2_reg[6] <= s1_reg[12];
            end
            
            // -----------------------------------------------------------------
            // Stage 3: 7개 -> 4개
            // -----------------------------------------------------------------
            if (valid_s2) begin
                s3_reg[0] <= max2(s2_reg[0], s2_reg[1]);
                s3_reg[1] <= max2(s2_reg[2], s2_reg[3]);
                s3_reg[2] <= max2(s2_reg[4], s2_reg[5]);
                s3_reg[3] <= s2_reg[6];
            end
            
            // -----------------------------------------------------------------
            // Stage 4: 4개 -> 1개 (최종 출력)
            // -----------------------------------------------------------------
            if (valid_s3) begin
                max_out <= max2( max2(s3_reg[0], s3_reg[1]), max2(s3_reg[2], s3_reg[3]) );
            end
        end
    end
endmodule