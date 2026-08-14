`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: pool_engine (maxpool 5x5 실행 엔진) - 격자 직접구동 재설계
//
// 역할: Top FSM 이 pool 스텝(SPPF maxpool)을 만나면 기동. 한 maxpool 레이어 완주.
//   ★재설계★ wrapper_unpacker/coord_gen/stream_unpacker 우회.
//   pool_engine 이 격자 (W+2)x(H+2) 순서로 (fold데이터, px, py, emit) 직접생성하여
//   maxpool_block_5x5_folded 에 직접 공급. 출력 fold 를 빽빽 패킹 GFB write.
//
// 격자공급 원리 (maxpool pad2 좌표정렬):
//   - 격자 순회: gpy 0..H+1, gpx 0..W+1, gf 0..in_folds-1
//   - 실제픽셀(gpx<W && gpy<H): gfold=(gpy*W+gpx)*in_folds+gf, 해당 GFB word/slot 추출
//     (같은 word 연속 slot 은 1회 read 후 캐싱 재사용 -> 자원/대역 절약)
//   - 더미(gpx>=W || gpy>=H): 0 공급 (window_gen 이 경계 -128 처리)
//   - px/py=입력좌표(window 정렬용), emit=(출력좌표 gpx-2,gpy-2 유효시 수집대상)
//   - 이로써 데이터-좌표 1:1 정렬 (기존 워드단위 공급의 행경계 더미누락 버그 해결)
//////////////////////////////////////////////////////////////////////////////////
module pool_engine #(
    parameter DATA_WIDTH = 8,
    parameter FOLD_CH    = 16,
    parameter MAX_CH     = 64,      // SPPF maxpool 최대 채널 (BRAM 절약)
    parameter MAX_WIDTH  = 40,      // SPPF 입력 폭 (40x40)
    parameter GFB_WIDTH  = 2048,
    parameter ADDR_WIDTH = 14
)(
    input  wire clk, rst_n,
    input  wire        i_start,
    output reg         o_done,
    output reg         o_busy,
    // 디코드 필드
    input  wire [13:0] i_in_base,
    input  wire [13:0] i_out_base,
    input  wire [9:0]  i_total_ch_in,
    input  wire [19:0] i_target_pixel,
    input  wire [10:0] i_img_width,
    input  wire [10:0] i_img_height,
    // GFB read
    output reg  [ADDR_WIDTH-1:0] o_gfb_raddr,
    output reg         o_gfb_re,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,
    // GFB write
    output reg  [ADDR_WIDTH-1:0] o_gfb_waddr,
    output reg         o_gfb_we,
    output reg  [GFB_WIDTH-1:0]  o_gfb_wdata
);
    localparam PAD = 2;
    wire [4:0]  in_folds = i_total_ch_in[9:4];          // 입력 fold 수
    wire [10:0] W = i_img_width;
    wire [10:0] H = i_img_height;
    wire [10:0] GW = i_img_width + PAD;                  // 격자 폭 (W+2)
    wire [10:0] GH = i_img_height + PAD;                 // 격자 높이 (H+2)
    // [누산화] 실제 출력 fold(=target_pixel*in_folds). 레이어 내 상수이므로
    //   조합 곱셈을 매 사이클 두지 않고 S_IDLE 에서 1회 계산해 레지스터에 보관.
    reg  [31:0] total_folds;                             // 실제 출력 fold (W*H*in_folds)
    wire [19:0] row_stride_w = W * in_folds;             // 한 행당 fold 수(상수, S_IDLE 래치용)

    // ===== maxpool_block 직접구동 신호 =====
    reg  [DATA_WIDTH*FOLD_CH-1:0] fp_data;   // 128b fold 데이터
    reg         fp_valid;
    reg         fp_last_fold;
    reg  [10:0] fp_px, fp_py;
    reg         fp_emit;
    wire        mp_rout;                     // maxpool_block ready_out (backpressure)
    reg         mp_ready;                    // 하위에 항상 ready
    wire [127:0] mp_out;
    wire        mp_vout, mp_lf;

    maxpool_block_5x5_folded #(.DATA_WIDTH(DATA_WIDTH),.FOLD_CH(FOLD_CH),
                               .MAX_CH(MAX_CH),.MAX_WIDTH(MAX_WIDTH)) u_maxpool (
        .clk(clk),.rst_n(rst_n),.img_width(i_img_width),.img_height(i_img_height),
        .total_ch_in(i_total_ch_in),
        .folded_pixel_in(fp_data),.fp_valid_in(fp_valid),.fp_last_fold_in(fp_last_fold),
        .fp_px_in(fp_px),.fp_py_in(fp_py),.fp_emit_in(fp_emit),.ready_out(mp_rout),
        .ready_in(mp_ready),.max_out(mp_out),.valid_out(mp_vout),.last_fold_out(mp_lf)
    );

    // ===== 입력 격자공급 FSM =====
    //   gpy/gpx/gf 격자카운터. 실제픽셀이면 GFB word/slot 추출, 더미면 0.
    localparam S_IDLE=0, S_RD=1, S_LAT=2, S_FEED=3, S_DONE=4;
    reg [2:0]  state;
    reg [10:0] gpx, gpy;     // 격자 좌표
    reg [4:0]  gf;           // 현재 fold
    reg [GFB_WIDTH-1:0] word_buf;  // 캐싱된 워드
    reg [13:0] buf_word;     // word_buf 가 담고있는 word 주소
    reg        buf_valid;    // word_buf 유효
    reg [19:0] gfold;        // 현재 실제픽셀 gfold
    reg [13:0] need_word;    // 필요 word offset
    reg [3:0]  need_slot;    // 필요 slot
    reg        did_read;     // 이번 픽셀 fold 에서 GFB read 했는지
    // [누산화] (gpy*W+gpx)*in_folds 를 매 사이클 곱셈하지 않고 가산으로 유지.
    //   pix_base = 현재 격자픽셀(gpx,gpy) 의 fold 기준주소(=pixel_index*in_folds).
    //   row_base = 현재 행 첫 픽셀(gpx=0) 의 기준주소(=gpy*W*in_folds).
    //   gfold = pix_base + gf 로 곱셈 완전 제거(가산만).
    reg [19:0] pix_base, row_base;
    reg [19:0] row_stride;   // W*in_folds (S_IDLE 1회 래치)

    wire is_real = (gpx < W) && (gpy < H);
    // 출력좌표 (gpx-PAD, gpy-PAD) 유효성 (수집대상)
    wire signed [11:0] ox = $signed({1'b0,gpx}) - PAD;
    wire signed [11:0] oy = $signed({1'b0,gpy}) - PAD;
    wire emit_ok = (ox>=0 && ox<$signed({1'b0,W}) && oy>=0 && oy<$signed({1'b0,H}));

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE; o_done<=0; o_busy<=0; o_gfb_re<=0;
            fp_valid<=0; fp_data<=0; fp_last_fold<=0; fp_px<=0; fp_py<=0; fp_emit<=0;
            mp_ready<=1; gpx<=0; gpy<=0; gf<=0; buf_valid<=0; buf_word<=0; did_read<=0;
        end else begin
            o_done<=0; o_gfb_re<=0; fp_valid<=0;
            case(state)
                S_IDLE: if(i_start) begin
                    o_busy<=1; gpx<=0; gpy<=0; gf<=0; buf_valid<=0;
                    pix_base<=0; row_base<=0;
                    row_stride<=row_stride_w;             // W*in_folds 1회 계산
                    total_folds<=i_target_pixel*in_folds; // 출력 fold 수 1회 계산
                    state<=S_RD;
                end
                // 현재 격자픽셀의 fold gf 에 필요한 word 준비
                S_RD: begin
                    if(is_real) begin
                        // [누산화] gfold=(gpy*W+gpx)*in_folds+gf  ->  pix_base+gf (가산만)
                        gfold = pix_base + gf;
                        need_word = gfold[19:4];   // /16
                        need_slot = gfold[3:0];    // %16
                        if(buf_valid && buf_word==need_word) begin
                            did_read<=0; state<=S_FEED;          // 캐시 히트
                        end else begin
                            o_gfb_raddr<= i_in_base + need_word; o_gfb_re<=1;
                            did_read<=1;
                            state<=S_LAT;
                        end
                    end else begin
                        need_slot = 0; did_read<=0; state<=S_FEED;   // 더미
                    end
                end
                S_LAT: begin
                    state<=S_FEED;     // GFB read latency 1클럭 -> 다음클럭 rdata 유효
                end
                S_FEED: begin
                    // GFB read 한 경우: rdata 를 캐시에 적재
                    if(is_real && did_read) begin
                        word_buf<=i_gfb_rdata; buf_word<=need_word; buf_valid<=1;
                    end
                    // 공급 데이터 선택
                    if(is_real) begin
                        if(did_read)
                            fp_data<= i_gfb_rdata[need_slot*128 +: 128];  // 방금 읽은 rdata
                        else
                            fp_data<= word_buf[need_slot*128 +: 128];     // 캐시 히트
                    end else begin
                        fp_data<= 0;   // 더미
                    end
                    fp_valid<=1;
                    fp_px<=gpx; fp_py<=gpy;
                    fp_last_fold<=(gf==in_folds-1);
                    fp_emit<=emit_ok;
                    // 격자 카운터 진행 (+ pix_base/row_base 누산)
                    if(gf==in_folds-1) begin
                        gf<=0;
                        if(gpx==GW-1) begin
                            gpx<=0;
                            if(gpy==GH-1) state<=S_DONE;
                            else begin
                                gpy<=gpy+1;
                                // 다음 행 첫 픽셀: row_base += W*in_folds, pix_base=새 row_base
                                row_base<=row_base+row_stride;
                                pix_base<=row_base+row_stride;
                                state<=S_RD;
                            end
                        end else begin
                            gpx<=gpx+1;
                            pix_base<=pix_base+in_folds;  // 같은 행 다음 픽셀
                            state<=S_RD;
                        end
                    end else begin
                        gf<=gf+1; state<=S_RD;            // 같은 픽셀(다음 fold): pix_base 불변
                    end
                end
                S_DONE: begin
                    if(collected >= total_folds) begin o_done<=1; o_busy<=0; state<=S_IDLE; end
                end
            endcase
        end
    end

    // ===== maxpool 출력 수집 -> 빽빽 패킹 -> GFB write =====
    reg [GFB_WIDTH-1:0] out_word;
    reg [GFB_WIDTH-1:0] out_word_n;
    reg [19:0] out_fold_cnt;
    reg [19:0] collected;
    integer slot;
    always @(posedge clk) begin
        if(!rst_n) begin
            out_word<=0; out_fold_cnt<=0; collected<=0; o_gfb_we<=0; o_gfb_waddr<=0; o_gfb_wdata<=0;
        end else begin
            o_gfb_we<=0;
            if(state==S_IDLE && i_start) begin
                out_word<=0; out_fold_cnt<=0; collected<=0;
            end else if(mp_vout && mp_ready) begin
                slot = out_fold_cnt % 16;
                out_word_n = out_word;
                out_word_n[slot*128 +: 128] = mp_out;
                out_word <= out_word_n;
                if(slot == 15) begin
                    o_gfb_we<=1;
                    o_gfb_waddr<= i_out_base + (out_fold_cnt/16);
                    o_gfb_wdata<= out_word_n;
                end
                out_fold_cnt<=out_fold_cnt+1;
                collected<=collected+1;
            end
        end
    end
endmodule
