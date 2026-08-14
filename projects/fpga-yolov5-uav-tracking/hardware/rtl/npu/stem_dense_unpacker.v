// =====================================================================
// Module: stem_dense_unpacker
// 역할: GFB 에 dense 로 적재된 입력 이미지(통신 팀 포맷)를 읽어
//       24-bit RGB 픽셀 스트림(R[7:0] G[15:8] B[23:16])으로 변환.
//       wrapper_6x6.pixel_in 직결, 배압(ready) 지원.
// 포맷: 행마다 WORDS_PER_ROW 워드 정렬(앞 img_width*3 B 만 픽셀, 나머지 패딩),
//       픽셀 3B 인터리브(R,G,B), 워드 내 byte j = 비트[j*8+:8], 워드경계 straddle 허용.
// 자원: BRAM/DSP 0 (2워드 윈도우 + 바이트 mux).
// =====================================================================
module stem_dense_unpacker #(
    parameter GFB_WIDTH  = 2048,
    parameter ADDR_WIDTH = 14,
    parameter ADDR_WIDTH2_UNUSED = 0
)(
    input  wire                   clk, rst_n,
    input  wire                   i_start,
    input  wire [ADDR_WIDTH-1:0]  i_base,
    input  wire [10:0]            i_img_width, i_img_height,
    input  wire [4:0]             i_words_per_row,  // 행 stride(워드): 64->1, 640->8
    output reg                    o_re,
    output reg  [ADDR_WIDTH-1:0]  o_raddr,
    input  wire [GFB_WIDTH-1:0]   i_rdata,
    input  wire                   i_rvalid,
    output wire [23:0]            o_pixel,    // [7:0]=R [15:8]=G [23:16]=B
    output wire                   o_pvalid,
    input  wire                   i_pready,
    output reg                    o_done
);
    localparam BYTES_PER_WORD = GFB_WIDTH/8;   // 256
    localparam S_IDLE=0,S_WAIT0=1,S_WAIT1=2,S_EMIT=3,S_ADV=4,S_DONE=5;
    reg [2:0] state;
    reg [GFB_WIDTH-1:0] wcur, wnxt;
    reg [10:0] col, row;
    reg [3:0]  cwir;
    reg [ADDR_WIDTH-1:0] row_base;

    wire [12:0] byte_in_row = {col,1'b0} + col;      // 3*col
    wire [12:0] win_start   = {2'b0, cwir, 8'b0};     // 256*cwir
    wire [8:0]  bp          = byte_in_row[12:0] - win_start[12:0];
    wire [4095:0] win = {wnxt, wcur};

    // 조합 출력 (skew 방지: accept 시 전진)
    assign o_pixel  = win[ {bp,3'b0} +: 24 ];
    assign o_pvalid = (state==S_EMIT) && (bp < BYTES_PER_WORD[8:0]);
    wire accept = o_pvalid && i_pready;
    wire last_col = (col == i_img_width-1);
    wire last_row = (row == i_img_height-1);

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE; o_re<=0; o_done<=0; col<=0; row<=0; cwir<=0; row_base<=0; o_raddr<=0;
        end else begin
            o_re<=0; o_done<=0;
            case(state)
                S_IDLE: if(i_start) begin
                    col<=0; row<=0; cwir<=0; row_base<=i_base;
                    o_raddr<=i_base; o_re<=1; state<=S_WAIT0;
                end
                S_WAIT0: if(i_rvalid) begin wcur<=i_rdata; o_raddr<=row_base+1; o_re<=1; state<=S_WAIT1; end
                S_WAIT1: if(i_rvalid) begin wnxt<=i_rdata; cwir<=0; state<=S_EMIT; end
                S_EMIT: begin
                    if(bp >= BYTES_PER_WORD[8:0]) begin
                        // 윈도우 전진 (col 불변): wcur<=wnxt, 다음 워드 적재
                        wcur<=wnxt; cwir<=cwir+1'b1;
                        o_raddr<=row_base + cwir + 2; o_re<=1; state<=S_ADV;
                    end else if(accept) begin
                        if(last_col) begin
                            if(last_row) state<=S_DONE;
                            else begin
                                row<=row+1'b1; col<=0; cwir<=0;
                                row_base<=row_base + i_words_per_row;
                                o_raddr<=row_base + i_words_per_row; o_re<=1; state<=S_WAIT0;
                            end
                        end else col<=col+1'b1;
                    end
                end
                S_ADV: if(i_rvalid) begin wnxt<=i_rdata; state<=S_EMIT; end
                S_DONE: begin o_done<=1; state<=S_IDLE; end
            endcase
        end
    end
endmodule
