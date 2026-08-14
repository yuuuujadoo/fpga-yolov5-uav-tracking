`timescale 1ns / 1ps

// =================================================================================
// 모듈명: window_generator_5x5
// 설명: 
//   - 5줄의 데이터를 받아 5x5(25픽셀)의 거대한 윈도우 조각을 조립합니다.
//   - 💡 [패딩 수술 완료] 합성곱용 0이 아닌, Maxpool 전용 최솟값인 -128 (8'h80)을 패딩.
//   - 💡 [좌표 수술 완료] 자체 타이머로 추측하던 좌표를 버리고, 앞단에서 흘러온
//     px_in, py_in을 사용하여 100% 오차 없는 절대 좌표 제로(최소) 패딩을 수행합니다.
//   - 💡 [타이밍 수정 옵션A+B] 3200-bit window_out 생성을 2단 파이프라인으로 분할.
//        · 단A: next_taps(고팬아웃 last_fold_idx mux 결과) 캡처 + 좌표 경계 마스크 생성
//        · 단B: 마스크로 next_taps vs PAD 선택 후 window_out 래치 (단순 2:1 mux)
//        기존 1단[탭선택 mux → 좌표비교 → 거대배열 분배]의 직렬 고팬아웃 경로(fo~513,
//        ~10ns 배선)를 둘로 분할하고, 캡처 레지스터에 max_fanout 속성을 부여하여
//        단B로의 분배 팬아웃을 제한 -> setup 경로 단축. window 내용/연산순서 불변
//        (정수 max·좌표판정 동일), latency만 +1 증가(valid/ready 핸드셰이크로 흡수).
// =================================================================================

module window_generator_5x5 #(
    parameter DATA_WIDTH = 8,
    parameter FOLD_CH    = 16,  
    parameter MAX_WIDTH  = 20,  
    parameter MAX_CH     = 256  
)(
    input wire clk, rst_n,
    
    input wire [10:0] img_width, img_height,
    input wire [9:0]  total_ch_in, 
    
    input wire [DATA_WIDTH*FOLD_CH-1:0] row0_in, row1_in, row2_in, row3_in, row4_in,
    input wire valid_in, last_fold_in,
    input wire [10:0] px_in, py_in,
    input wire emit_in,
    
    input wire ready_in,   
    output wire ready_out, 
    
    output reg [DATA_WIDTH*FOLD_CH*25-1:0] window_out,
    output reg valid_out, last_fold_out,
    output reg [10:0] px_out, py_out
);

    assign ready_out = ready_in;
    wire [DATA_WIDTH*FOLD_CH-1:0] PAD_VAL = {FOLD_CH{8'h80}}; 

    reg [DATA_WIDTH*FOLD_CH-1:0] delay_lines [0:4][0:3][0:15];
    wire [4:0] current_folds = total_ch_in / FOLD_CH; 
    wire [3:0] last_fold_idx = (current_folds==0) ? 4'd0 : (current_folds[4:0] - 5'd1); // iverilog12 폭단언 우회용 4bit 인덱스

    integer r, t, d;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (r=0; r<5; r=r+1)
                for (t=0; t<4; t=t+1)
                    for (d=0; d<16; d=d+1)
                        delay_lines[r][t][d] <= 0;
        end 
        else if (ready_in && valid_in) begin 
            for (r=0; r<5; r=r+1) begin
                delay_lines[r][3][0] <= (r==0)? row0_in : (r==1)? row1_in : 
                                        (r==2)? row2_in : (r==3)? row3_in : row4_in;
                for (d=1; d<16; d=d+1) delay_lines[r][3][d] <= delay_lines[r][3][d-1];
                
                delay_lines[r][2][0] <= delay_lines[r][3][last_fold_idx];
                for (d=1; d<16; d=d+1) delay_lines[r][2][d] <= delay_lines[r][2][d-1];
                
                delay_lines[r][1][0] <= delay_lines[r][2][last_fold_idx];
                for (d=1; d<16; d=d+1) delay_lines[r][1][d] <= delay_lines[r][1][d-1];
                
                delay_lines[r][0][0] <= delay_lines[r][1][last_fold_idx];
                for (d=1; d<16; d=d+1) delay_lines[r][0][d] <= delay_lines[r][0][d-1];
            end
        end
    end

    wire [DATA_WIDTH*FOLD_CH-1:0] next_taps [0:4][0:4];
    genvar gr;
    generate
        for (gr=0; gr<5; gr=gr+1) begin : tap_assign
            assign next_taps[gr][4] = (gr==0)? row0_in : (gr==1)? row1_in : 
                                      (gr==2)? row2_in : (gr==3)? row3_in : row4_in;
            assign next_taps[gr][3] = delay_lines[gr][3][last_fold_idx];
            assign next_taps[gr][2] = delay_lines[gr][2][last_fold_idx];
            assign next_taps[gr][1] = delay_lines[gr][1][last_fold_idx];
            assign next_taps[gr][0] = delay_lines[gr][0][last_fold_idx];
        end
    endgenerate

    // =========================================================================
    // [옵션 B 단A] next_taps 캡처 + 좌표 경계 마스크 생성  (한 클록)
    //   - tap_reg : 고팬아웃 탭선택(mux) 결과를 레지스터로 흡수
    //   - mask_reg: 25개 윈도우 위치 각각의 경계 유효 여부(1=실데이터,0=PAD)
    //   - (* max_fanout *)로 단B 분배 팬아웃 제한 (옵션 A)
    // =========================================================================
    (* max_fanout = 64 *) reg [DATA_WIDTH*FOLD_CH-1:0] tap_reg [0:4][0:4];
    (* max_fanout = 64 *) reg [24:0] mask_reg;
    reg valid_a, last_fold_a, emit_a;
    reg [10:0] px_a, py_a;

    integer rr, cc;
    reg signed [12:0] ax, ay;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_a <= 0; last_fold_a <= 0; emit_a <= 0; px_a <= 0; py_a <= 0;
            mask_reg <= 0;
            for (rr=0; rr<5; rr=rr+1)
                for (cc=0; cc<5; cc=cc+1)
                    tap_reg[rr][cc] <= 0;
        end
        else if (ready_in) begin
            valid_a     <= valid_in;
            last_fold_a <= last_fold_in;
            emit_a      <= emit_in;
            px_a        <= px_in;
            py_a        <= py_in;
            if (valid_in) begin
                for (rr=0; rr<5; rr=rr+1) begin
                    for (cc=0; cc<5; cc=cc+1) begin
                        ax = $signed({1'b0, px_in}) - 4 + cc;
                        ay = $signed({1'b0, py_in}) - 4 + rr;
                        tap_reg[rr][cc]   <= next_taps[rr][cc];
                        mask_reg[rr*5+cc] <= (ax >= 0 && ax < $signed({1'b0, img_width}) &&
                                              ay >= 0 && ay < $signed({1'b0, img_height}));
                    end
                end
            end
        end
    end

    // =========================================================================
    // [옵션 B 단B] 마스크 선택 + window_out 래치  (단순 2:1 mux, 한 클록)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            window_out <= 0; valid_out <= 0; last_fold_out <= 0; px_out <= 0; py_out <= 0;
        end 
        else if (ready_in) begin 
            valid_out     <= valid_a && emit_a;
            last_fold_out <= last_fold_a && emit_a;
            px_out        <= px_a;
            py_out        <= py_a;
            if (valid_a) begin
                for (rr=0; rr<5; rr=rr+1) begin
                    for (cc=0; cc<5; cc=cc+1) begin
                        window_out[((rr*5 + cc)*DATA_WIDTH*FOLD_CH) +: (DATA_WIDTH*FOLD_CH)]
                            <= mask_reg[rr*5+cc] ? tap_reg[rr][cc] : PAD_VAL;
                    end
                end
            end
        end
    end

endmodule
