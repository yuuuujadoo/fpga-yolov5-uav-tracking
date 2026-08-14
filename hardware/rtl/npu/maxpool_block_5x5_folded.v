`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: maxpool_block_5x5_folded  (공용 폴더 외부화 리팩토링판)
//
// Description:
//   - 기존 maxpool_block_5x5 에서 내부 channel_folder_5x5 인스턴스를 '제거'하고,
//     Top 의 '공용 폴더(= wrapper_unpacker)'가 만든 folded 스트림을 직접 입력받는다.
//   - 제거 이유:
//     (1) channel_folder_5x5 는 wrapper_unpacker(stream_unpacker+coord_generator)와
//         역할이 100% 중복(2048b 픽셀 -> 128b fold + px/py 좌표 생성).
//     (2) channel_folder_5x5 도 동일한 zero-bubble 구조라 C3 검증에서 확인된
//         "chunk15 연속공급 시 마지막 fold 유실" 위험을 그대로 가짐.
//     (3) Top 에서 conv(1x1/3x3)·maxpool 이 단일 공용 폴더를 공유 -> BRAM/로직 절감 +
//         검증된 단일 폴딩 경로로 일원화.
//
//   - 입력 인터페이스 변경:
//       기존:  pixel_in[2048], valid_in        (+ 내부 폴더)
//       변경:  folded_pixel_in[128], fp_valid_in, fp_last_fold_in, fp_px_in, fp_py_in
//              (= wrapper_unpacker 의 data_out/valid_out/last_fold_out/px_out/py_out 직결)
//   - 나머지(line_buffer -> window_gen -> 1clk delay -> max_array)는 원본과 동일.
//   - ready_out 은 line_buffer 의 ready_out 을 그대로 노출(공용 폴더로의 backpressure).
//////////////////////////////////////////////////////////////////////////////////

module maxpool_block_5x5_folded #(
    parameter DATA_WIDTH = 8,
    parameter MAX_CH     = 256,
    parameter MAX_WIDTH  = 20,
    parameter FOLD_CH    = 16
)(
    input wire clk, rst_n,

    input wire [10:0] img_width, img_height,
    input wire [9:0]  total_ch_in,

    // === 외부 공용 폴더(wrapper_unpacker) 출력 직결 ===
    input wire [DATA_WIDTH*FOLD_CH-1:0] folded_pixel_in,  // unpacker.data_out
    input wire                          fp_valid_in,        // unpacker.valid_out
    input wire                          fp_last_fold_in,    // unpacker.last_fold_out
    input wire [10:0]                   fp_px_in,           // unpacker.px_out
    input wire [10:0]                   fp_py_in,           // unpacker.py_out
    input wire                          fp_emit_in,         // unpacker.emit_out (수집대상 플래그)
    output wire                         ready_out,          // -> unpacker.ready_in (backpressure)

    input wire ready_in,
    output wire [DATA_WIDTH*FOLD_CH-1:0] max_out,
    output wire valid_out,
    output wire last_fold_out
);

    // =========================================================================
    // [1] Line Buffer  (입력 = 외부 folded 스트림 직결)
    // =========================================================================
    wire [DATA_WIDTH*FOLD_CH-1:0] lb_row0, lb_row1, lb_row2, lb_row3, lb_row4;
    wire lb_valid, lb_last_fold, wg_ready_out;
    wire [10:0] lb_px, lb_py;
    wire lb_emit;

    line_buffer_5x5 #(
        .DATA_WIDTH(DATA_WIDTH), .FOLD_CH(FOLD_CH), .MAX_WIDTH(MAX_WIDTH), .MAX_CH(MAX_CH)
    ) u_line_buf (
        .clk(clk), .rst_n(rst_n),
        .img_width(img_width), .total_ch_in(total_ch_in),
        .pixel_in(folded_pixel_in), .valid_in(fp_valid_in), .last_fold_in(fp_last_fold_in),
        .px_in(fp_px_in), .py_in(fp_py_in), .emit_in(fp_emit_in),
        .ready_in(wg_ready_out), .ready_out(ready_out),   // ready_out -> 공용 폴더로
        .row0_out(lb_row0), .row1_out(lb_row1), .row2_out(lb_row2),
        .row3_out(lb_row3), .row4_out(lb_row4),
        .valid_out(lb_valid), .last_fold_out(lb_last_fold),
        .px_out(lb_px), .py_out(lb_py), .emit_out(lb_emit)
    );

    // =========================================================================
    // [2] Window Generator
    // =========================================================================
    wire [DATA_WIDTH*FOLD_CH*25-1:0] wg_window;
    wire wg_valid, wg_last_fold;
    wire [10:0] wg_px, wg_py;

    window_generator_5x5 #(
        .DATA_WIDTH(DATA_WIDTH), .FOLD_CH(FOLD_CH), .MAX_WIDTH(MAX_WIDTH), .MAX_CH(MAX_CH)
    ) u_win_gen (
        .clk(clk), .rst_n(rst_n),
        .img_width(img_width), .img_height(img_height),
        .total_ch_in(total_ch_in),
        .row0_in(lb_row0), .row1_in(lb_row1), .row2_in(lb_row2),
        .row3_in(lb_row3), .row4_in(lb_row4),
        .valid_in(lb_valid), .last_fold_in(lb_last_fold),
        .px_in(lb_px), .py_in(lb_py), .emit_in(lb_emit),
        .ready_in(ready_in), .ready_out(wg_ready_out),
        .window_out(wg_window), .valid_out(wg_valid), .last_fold_out(wg_last_fold),
        .px_out(wg_px), .py_out(wg_py)
    );

    // =========================================================================
    // [3] 파이프라인 마진 레지스터 (1-Clock Delay)
    // =========================================================================
    reg [DATA_WIDTH*FOLD_CH*25-1:0] delayed_window;
    reg delayed_valid, delayed_last_fold;
    always @(posedge clk) begin
        if (!rst_n) begin
            delayed_window <= 0; delayed_valid <= 0; delayed_last_fold <= 0;
        end else if (ready_in) begin
            delayed_window    <= wg_window;
            delayed_valid     <= wg_valid;
            delayed_last_fold <= wg_last_fold;
        end
    end

    // =========================================================================
    // [4] Maxpool 5x5 Array
    // =========================================================================
    // px/py 5단 지연 (wg출력 -> delayed(1) -> max_array(4) = valid_out 정렬)
    reg [10:0] px_d [0:4];
    reg [10:0] py_d [0:4];
    integer k;
    always @(posedge clk) begin
        if(!rst_n) begin
            for(k=0;k<5;k=k+1) begin px_d[k]<=0; py_d[k]<=0; end
        end else if(ready_in) begin
            px_d[0]<=wg_px; py_d[0]<=wg_py;
            for(k=1;k<5;k=k+1) begin px_d[k]<=px_d[k-1]; py_d[k]<=py_d[k-1]; end
        end
    end
    // 출력 좌표 (px-2,py-2) 유효성 (maxpool pad2)
    wire signed [11:0] mox = $signed({1'b0,px_d[4]}) - 2;
    wire signed [11:0] moy = $signed({1'b0,py_d[4]}) - 2;
    wire out_emit = (mox>=0 && mox<$signed({1'b0,img_width}) && moy>=0 && moy<$signed({1'b0,img_height}));

    wire ma_valid, ma_last_fold;
    maxpool_5x5_array #(
        .DATA_WIDTH(DATA_WIDTH), .FOLD_CH(FOLD_CH)
    ) u_max_array (
        .clk(clk), .rst_n(rst_n),
        .window_in(delayed_window), .valid_in(delayed_valid), .last_fold_in(delayed_last_fold),
        .ready_in(ready_in),
        .pixel_out(max_out), .valid_out(ma_valid), .last_fold_out(ma_last_fold)
    );
    // 최종 출력: 출력대상(out_emit) 픽셀만 valid (격자 패딩픽셀 마스킹)
    assign valid_out     = ma_valid & out_emit;
    assign last_fold_out = ma_last_fold & out_emit;

endmodule
