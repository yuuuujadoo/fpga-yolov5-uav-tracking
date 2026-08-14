`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: conv_engine (스트리밍 재설계 - 1x1 우선)
//
// 변경점 (이전 버퍼 방식 -> 스트리밍):
//   - unpacker/feeder 제거: conv_block 은 2048b 픽셀 통째를 받으므로(내부 channel_folder
//     가 fold 처리) unpacker 불필요. GFB 워드에서 픽셀을 직접 추출.
//   - 입력 전체 버퍼(pixbuf 전체 npix) 제거 -> 타일 버퍼(TILE_PIX 픽셀)로 축소.
//     그룹 재사용을 위해 타일 단위로 처리: 타일 입력 적재 -> 그 타일의 모든 그룹 conv
//     -> 다음 타일.
//   - 출력 전체 버퍼(outbuf 전체) 제거 -> 출력 1워드 레지스터로 즉시 GFB write.
//
// BRAM: 타일버퍼 TILE_PIX*2048b (작게). 출력 레지스터 2048b(FF).
//
// 검증된 규약 유지: weight 2클럭 지연(1x1), 그룹경계 reset, 빽빽 패킹 출력.
//
// GFB 빽빽 패킹: 1워드=16fold. 픽셀 p 의 fold f -> 전역fold=p*INF+f,
//   word=gf//16, slot=gf%16. 픽셀 추출 = 연속 INF 슬롯 모음.
//////////////////////////////////////////////////////////////////////////////////
module conv_engine #(
    parameter DATA_WIDTH = 8,
    parameter MAX_CH     = 256,
    parameter T0         = 16,
    parameter FOLD_CH    = 16,
    parameter GFB_WIDTH  = 2048,
    parameter ADDR_WIDTH = 14,
    parameter TILE_PIX   = 64,      // 타일당 픽셀 수 (BRAM/throughput 조절)
    parameter WLAT       = 2,       // weight 지연 단수 (조합ROM=2, 1클럭ROM=1)
    parameter PLAT       = 1,       // bias 지연 (조합ROM=1, 1클럭ROM=0)
    parameter PLAT_PP    = 0        // M0/n 지연 (조합ROM=0 직접, 1클럭ROM=0)
)(
    input  wire clk, rst_n,
    input  wire        i_start,
    output reg         o_done,
    output reg         o_busy,
    // 디코드 필드
    input  wire [13:0] i_in_base,
    input  wire [13:0] i_out_base,
    input  wire [9:0]  i_total_ch_in,
    input  wire [9:0]  i_out_ch,
    input  wire [19:0] i_target_pixel,
    input  wire [10:0] i_img_width,
    input  wire [10:0] i_img_height,
    input  wire [1:0]  i_stride,
    input  wire        i_is_1x1,
    input  wire        i_is_layer_0,
    input  wire [19:0] i_src_pixel,   // stem 입력 픽셀수 (img_w*img_h)
    input  wire        i_silu_en,
    // weight/param 요청
    output reg  [4:0]  o_req_grp,
    output wire [4:0]  o_req_fold,
    input  wire [T0*DATA_WIDTH*FOLD_CH-1:0]   i_weight,
    input  wire [T0*DATA_WIDTH*FOLD_CH*9-1:0] i_weight3,
    input  wire [T0*16-1:0] i_bias,
    input  wire [T0*16-1:0] i_M0,
    input  wire [T0*5-1:0]  i_n,
    // GFB read
    output wire [ADDR_WIDTH-1:0] o_gfb_raddr,
    output wire        o_gfb_re,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,
    // GFB write
    output wire [ADDR_WIDTH-1:0] o_gfb_waddr,
    output wire        o_gfb_we,
    output wire [GFB_WIDTH-1:0]  o_gfb_wdata
);
    wire [4:0] in_folds_raw = i_total_ch_in[9:4];   // INF (16ch=1)
    wire [4:0] in_folds = i_is_layer_0 ? 5'd1 : (in_folds_raw==0 ? 5'd1 : in_folds_raw);
    wire [4:0] out_grps = i_out_ch[9:4];        // OUTG
    // 3x3 은 line buffer warmup 위해 픽셀 추가 공급 (수집은 tile_n 까지)
    wire [19:0] feed_extra = i_is_1x1 ? 20'd0 : (i_img_width + 2);

    // ===== 전방 선언: 아래 인스턴스 포트에서 '선언보다 먼저' 쓰이는 신호들 =====
    //   (Verilog 의 묵시적 1비트 net 생성 방지 -> 시뮬/합성 동작 일치 보장)
    wire        dw_ready;            // backpressure (RMW 중 0)
    reg  [T0*16-1:0] c_M0; reg [T0*5-1:0] c_n;
    reg         cv3; reg [GFB_WIDTH-1:0] cpix3;
    wire        o1_busy;             // 1x1 collect RMW 중 backpressure
    reg  [4:0]  grp;

    // ===== 타일 입력 버퍼 (TILE_PIX 픽셀, 2048b 픽셀 형태로 저장) =====
    reg [GFB_WIDTH-1:0] tilebuf [0:TILE_PIX-1];
    reg [19:0] tile_base;     // 현재 타일의 시작 픽셀 인덱스
    reg [9:0]  tile_n;        // 현재 타일의 픽셀 수 (<=TILE_PIX)

    // ===== conv_block_1x1 (엔진 소유) =====
    reg                       grp_reset;
    reg                       c_val;
    reg  [GFB_WIDTH-1:0]      c_pix;
    reg  [T0*DATA_WIDTH*FOLD_CH-1:0] c_w;
    reg  [T0*16-1:0]          c_b;
    wire                      pp_rout;
    wire [T0*32-1:0] c1_psum; wire c1_vout, c1_rout; wire [4:0] c1_fidx;
    conv_block_1x1 #(.DATA_WIDTH(DATA_WIDTH),.MAX_CH(MAX_CH),.T0(T0),.FOLD_CH(FOLD_CH)) u_conv1 (
        .clk(clk),.rst_n(rst_n & ~grp_reset),
        .total_ch_in(i_total_ch_in),.pixel_in(c_pix),.valid_in(c_val),
        .weight_in(c_w),.bias_in(c_b),.psum_out(c1_psum),.valid_out(c1_vout),
        .ready_in(pp_rout),.ready_out(c1_rout),.fold_idx_out(c1_fidx)
    );
    // ===== stem 6x6 프론트엔드 (wrapper_6x6) =====
    //   stem(is_layer_0): GFB 픽셀 순차공급 -> 24b RGB 추출 -> wrapper_6x6
    //   -> 864b 윈도우 -> {288'd0,win}=1152b -> conv_block_3x3 ext_window (is_layer_0=1)
    wire [23:0] stem_pix24; wire stem_pvalid;      // wrapper_6x6 입력: unpacker/flush mux
    // ===== dense 입력 언패커 연결 (stem 전용) =====
    wire [23:0] du_pixel; wire du_pvalid, du_done; wire du_re; wire [ADDR_WIDTH-1:0] du_raddr;
    wire stem_wrdy;                                 // wrapper_6x6 ready_out (배압)
    reg du_start, stem_flush, stem_flush_valid; reg [19:0] flush_p; reg du_re_d;
    wire [4:0] words_per_row = ((i_img_width*3) + 11'd255) >> 8;  // 행 stride 워드수(64->1,640->8)
    assign stem_pix24  = stem_flush ? 24'd0 : du_pixel;
    assign stem_pvalid = stem_flush ? stem_flush_valid : du_pvalid;
    wire du_pready = stem_wrdy & ~stem_flush;
    wire [863:0] stem_win864; wire stem_wvalid;     // wrapper_6x6 출력

    wire [1151:0] stem_win1152 = {288'd0, stem_win864};
    // [전략C] conv_block.ready_out(c3_rout) 선언을 앞으로 이동:
    //   stem(is_layer_0)일 때 c3_rout = pp_rout & stem_sub 가 되어, wrapper_6x6 를
    //   2 cycle 마다 advance 시켜 stem window 를 2 cycle hold(8채널 sub-fold x2 처리)함.
    wire c3_rout;
    wrapper_6x6 #(.DATA_WIDTH(24),.MAX_WIDTH(640),.WINDOW_BITS(864)) u_stem6 (
        .clk(clk),.rst_n(rst_n & ~grp_reset),
        .img_width(i_img_width),.img_height(i_img_height),.stride(i_stride),
        .pixel_in(stem_pix24),.valid_in(stem_pvalid),
        .ready_in(c3_rout),.ready_out(stem_wrdy),
        .window_out(stem_win864),.valid_out(stem_wvalid)
    );
    // dense 입력 언패커: GFB -> 24b RGB 스트림 (GFB 1clk latency -> du_re_d 가 rvalid)
    always @(posedge clk) if(!rst_n) du_re_d<=1'b0; else du_re_d<=du_re;
    stem_dense_unpacker #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH)) u_du (
        .clk(clk),.rst_n(rst_n),.i_start(du_start),.i_base(i_in_base),
        .i_img_width(i_img_width),.i_img_height(i_img_height),.i_words_per_row(words_per_row),
        .o_re(du_re),.o_raddr(du_raddr),.i_rdata(i_gfb_rdata),.i_rvalid(du_re_d),
        .o_pixel(du_pixel),.o_pvalid(du_pvalid),.i_pready(du_pready),.o_done(du_done)
    );

    // conv_block_3x3 (다음 단계에서 스트리밍 통합; 지금은 1x1 경로 검증)
    wire [T0*32-1:0] c3_psum; wire c3_vout; wire [4:0] c3_fidx;  // c3_rout 은 위(u_stem6 앞)에서 선언
    wire [T0*DATA_WIDTH*FOLD_CH*9-1:0] c3_w;
    conv_block_3x3 #(.DATA_WIDTH(DATA_WIDTH),.MAX_CH(MAX_CH),.T0(T0),.FOLD_CH(FOLD_CH),.WLAT(WLAT)) u_conv3 (
        .clk(clk),.rst_n(rst_n & ~grp_reset),
        .img_width(i_img_width),.img_height(i_img_height),.total_ch_in(i_total_ch_in),
        .stride(i_stride),.is_layer_0(i_is_layer_0),.ext_window_in(stem_win1152),.ext_valid_in(stem_wvalid),
        .pixel_in(cpix3),.valid_in(cv3),
        .weight_in(c3_w),.bias_in(c_b),.psum_out(c3_psum),.valid_out(c3_vout),
        .ready_out(c3_rout),.fold_idx_out(c3_fidx),.ready_in(pp_rout)
    );
    wire [T0*32-1:0] c_psum = i_is_1x1 ? c1_psum : c3_psum;
    wire c_vout = i_is_1x1 ? c1_vout : c3_vout;
    wire c_rout = i_is_1x1 ? c1_rout : c3_rout;
    wire [4:0] c_fidx = i_is_1x1 ? c1_fidx : c3_fidx;
    // 3x3 weight: stall(pp_rout=0) 시 weight ROM 출력(i_weight3) 이 fold 주소진행으로
    //   어긋나 c3_w 가 한 fold 건너뛰는 문제(1x1 과 동일) -> stall-pending 방식으로 해결.
    //   stall 진입 첫클럭의 i_weight3 를 보존, stall 해제시 우선 사용 -> fold 누락 방지.
    wire [T0*DATA_WIDTH*FOLD_CH*9-1:0] c3_w_raw = (c3_fidx < in_folds) ? i_weight3 : {(T0*DATA_WIDTH*FOLD_CH*9){1'b0}};
    reg  [T0*DATA_WIDTH*FOLD_CH*9-1:0] c3_w_frz;     // stall 동안 c3_w hold 값
    reg  [T0*DATA_WIDTH*FOLD_CH*9-1:0] c3_w_pend;    // stall 진입시 놓칠 weight 보존
    reg  c3_pend_v;
    reg  c3_prout_d;
    always @(posedge clk) begin
        if(!rst_n) begin c3_w_frz<=0; c3_w_pend<=0; c3_pend_v<=0; c3_prout_d<=1; end
        else begin
            c3_prout_d<=pp_rout;
            // stall 진입 첫클럭(직전 ready, 현재 stall): 현재 c3_w_raw 를 pending 보존
            if(c3_prout_d && !pp_rout) begin c3_w_pend<=c3_w_raw; c3_pend_v<=1; end
            if(pp_rout) begin c3_w_frz<=c3_w_raw; c3_pend_v<=0; end
        end
    end
    // stall(pp_rout=0): freeze 값. ready: pending 우선(stall때 놓친 weight) else 직통.
    assign c3_w = pp_rout ? (c3_pend_v ? c3_w_pend : c3_w_raw) : c3_w_frz;

    // post_processor
    reg out_ready;
    wire [T0*DATA_WIDTH-1:0] pp_pix; wire pp_vout;
    post_processor #(.T0(T0)) u_pp (
        .clk(clk),.rst_n(rst_n & ~grp_reset),
        .M0_in(c_M0),.n_in(c_n),.silu_en(i_silu_en),
        .psum_in(c_psum),.valid_in(c_vout),.ready_in(out_ready & (i_is_1x1 ? ~o1_busy : dw_ready)),
        .ready_out(pp_rout),.pixel_out(pp_pix),.valid_out(pp_vout)
    );

    // ===== data_writer: collect + 빽빽패킹 + RMW (모든 op 공유) =====
    reg         dw_start;            // 그룹 시작 펄스
    wire        dw_we; wire [ADDR_WIDTH-1:0] dw_waddr; wire [GFB_WIDTH-1:0] dw_wdata;
    wire        dw_rmw_re; wire [ADDR_WIDTH-1:0] dw_rmw_raddr;
    wire        dw_grp_done; wire dw_flush_done;
    // data_writer 의 픽셀 입력: post_processor 출력 (모든 op 공통)
    data_writer #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.T0(T0),.DATA_WIDTH(DATA_WIDTH)) u_dw (
        .clk(clk),.rst_n(rst_n),
        .i_start(dw_start),
        .i_out_base(i_out_base),
        .i_target_pixel(i_target_pixel),
        .i_out_grps(out_grps),
        .i_grp(grp),
        .i_pix(pp_pix),.i_valid(pp_vout),.o_ready(dw_ready),
        .o_we(dw_we),.o_waddr(dw_waddr),.o_wdata(dw_wdata),
        .o_rmw_re(dw_rmw_re),.o_rmw_raddr(dw_rmw_raddr),.i_gfb_rdata(i_gfb_rdata),
        .o_grp_done(dw_grp_done),.o_flush_done(dw_flush_done)
    );
    assign o_req_fold = c_fidx;

    // weight 지연 (WLAT 단). 조합 ROM=2, 실제 1클럭 ROM=1 (ROM 이 1단 대체).
    // stall 시 weight ROM 정렬붕괴 수정: stall 진입 첫클럭의 i_weight(=다음 fold
    //   weight) 를 pending 에 보존, stall 해제시 그것을 우선 사용하여 fold 누락 방지.
    reg [T0*DATA_WIDTH*FOLD_CH-1:0] w_dly;
    reg [T0*DATA_WIDTH*FOLD_CH-1:0] w_pend;
    reg w_pend_v;
    reg prout_d;
    always @(posedge clk) begin
        if(!rst_n) begin w_dly<=0; c_w<=0; w_pend<=0; w_pend_v<=0; prout_d<=1; end
        else begin
            prout_d<=pp_rout;
            // stall 진입 첫클럭(prev ready, now stall): 현재 i_weight 를 pending 보존
            if(prout_d && !pp_rout) begin w_pend<=i_weight; w_pend_v<=1; end
            if(pp_rout) begin
                // stall 해제후 첫 캡처: pending(stall때 놓친 weight) 우선
                c_w <= w_pend_v ? w_pend : i_weight;
                w_pend_v<=0;
            end
        end
    end
    // bias 지연 (PLAT): 조합ROM=1클럭, 실제ROM=0(ROM이 1클럭 담당)
    generate
      if(PLAT>=1) begin: g_bdly
        always @(posedge clk) c_b<=i_bias;
      end else begin: g_bcomb
        always @(*) c_b=i_bias;
      end
    endgenerate
    // M0/n 지연 (PLAT_PP): 단독 조합=0(직접). Top 은 ROM 1클럭 이미 있음 -> 0 직접.
    generate
      if(PLAT_PP>=1) begin: g_ppdly
        always @(posedge clk) begin c_M0<=i_M0; c_n<=i_n; end
      end else begin: g_ppcomb
        always @(*) begin c_M0=i_M0; c_n=i_n; end
      end
    endgenerate

    // ===== 출력 1워드 누적 레지스터 (빽빽 패킹) =====
    reg [GFB_WIDTH-1:0] out_word;
    reg m_re; reg [ADDR_WIDTH-1:0] m_raddr;
    reg m_we; reg [ADDR_WIDTH-1:0] m_waddr; reg [GFB_WIDTH-1:0] m_wdata;
    reg [13:0] out_word_addr;   // 현재 누적중 워드의 GFB 주소

    // ===== 메인 FSM 및 1x1 RMW 상태 선언 (앞쪽 always 블록에서 참조) =====
    localparam S_IDLE=0,S_TILE_RD=1,S_TILE_LAT=2,S_TILE_WAIT=3,S_GRP_INIT=4,S_FEED=5,S_FLUSH=6,S_DONE=7,
               S3_GINIT=8,S3_RUN=9,S3_FLUSH=10,
               ST_GINIT=11,ST_RD=12,ST_LAT=13,ST_FEED=14,ST_FLUSH=15,ST_WAIT=17,
               S_TILE_WR=16;   // tilebuf 직렬 적재(1 full-word/clk, BRAM 추론용)
    reg [4:0] state;   // stem 상태(~17) 수용 위해 5비트
    localparam O1_IDLE=0,O1_RMWR=1,O1_MERGE=2,O1_FRMW=3,O1_FMERGE=4;
    reg [2:0] o1_state;

    // ========================================================================
    // 3x3 행 스트리밍: prefetch FIFO + supply + collect (모두 별도 always)
    //   read/supply 디커플링으로 GFB latency 와 ready 핸드셰이크 분리.
    // ========================================================================
    localparam FF_DEPTH = 32;                // 픽셀 FIFO 깊이 (워드당 다중픽셀 수용)
    (* ram_style = "distributed" *) reg  [GFB_WIDTH-1:0] ff_mem [0:FF_DEPTH-1];  // 얕음(32) -> LUTRAM, BRAM 절약
    reg  [5:0] ff_wr, ff_rd;                 // write/read 포인터 (wrap)
    reg re3; reg [ADDR_WIDTH-1:0] raddr3;
    reg [GFB_WIDTH-1:0] ow_3;
    reg [19:0] st_p; reg [23:0] stem_rgb;
    reg o_re_st; reg [ADDR_WIDTH-1:0] o_gfb_raddr_st;
    reg [5:0] ff_rd_st;  // stem 전용 FIFO read 포인터
    wire [5:0] ff_rd_eff = i_is_layer_0 ? ff_rd_st : ff_rd;
    wire [5:0] ff_cnt = ff_wr - ff_rd_eff;       // 채워진 개수
    wire ff_full  = (ff_cnt >= FF_DEPTH);   // 8칸 다 차면 full
    wire ff_empty = (ff_cnt == 0);

    // --- prefetch 상태 (read 전담) ---
    localparam PF_IDLE=0, PF_RD=1, PF_LAT=2, PF_PUSH=3;
    reg [1:0] pf_state;
    reg [19:0] pf_p;          // 다음 읽을 픽셀 인덱스
    reg [GFB_WIDTH-1:0] pf_word;
    reg pf_active;            // 현재 그룹 prefetch 진행중

    integer ps, pw;
    reg [4:0] px_per_word;   // [TIMING] 16/in_folds 를 S_IDLE 에서 1회 계산(가변 나눗셈을 per-word 경로에서 제거)
    // 공급할 입력 픽셀수: stem/stride2 는 i_src_pixel(입력), 아니면 i_target_pixel
    wire [19:0] in_pixels = (i_is_layer_0 || i_stride==2) ? i_src_pixel : i_target_pixel;
    wire [19:0] pf_limit = in_pixels;
    reg [GFB_WIDTH-1:0] ff_tmp;
    reg [5:0] pfw;                 // PF_PUSH 워드내 픽셀 직렬 카운터
    reg [GFB_WIDTH-1:0] word_src;  // 조합: 적재 소스 선택
    always @(posedge clk) begin
        if(!rst_n) begin
            pf_state<=PF_IDLE; pf_p<=0; pf_active<=0; re3<=0; ff_wr<=0;
        end else begin
            re3<=0;
            // 그룹 시작 신호: S3_GINIT 에서 pf 리셋
            if(i_is_layer_0) begin pf_active<=0; end
            else if(state==S3_GINIT) begin pf_p<=0; pf_active<=1; pf_state<=PF_IDLE; ff_wr<=0; end
            else if(pf_active) begin
                case(pf_state)
                    PF_IDLE: begin
                        // RMW(collect) 가 GFB read 포트 사용중이면 prefetch read 보류 (경합 방지)
                        if(dw_rmw_re) begin
                            re3<=0;  // 양보
                        end else if((ff_cnt + px_per_word <= FF_DEPTH) && pf_p < pf_limit) begin
                            raddr3<= i_in_base + (pf_p*in_folds)/16; re3<=1;
                            pf_state<=PF_LAT;
                        end else if(pf_p >= pf_limit) pf_active<=0; // 다 읽음
                    end
                    PF_LAT: begin
                        // RMW 가 read 포트 가로챘으면 이번 latency 무효 -> 재요청
                        if(dw_rmw_re) pf_state<=PF_IDLE;  // 재시도
                        else begin pfw<=0; pf_state<=PF_PUSH; end
                    end
                    PF_PUSH: begin
                        // 워드내 픽셀을 1개/클럭 직렬 push -> ff_mem 단일 write port (BRAM 추론 가능).
                        //  (기존: 한 클럭에 px_per_word 개 다중쓰기 -> BRAM 추론 실패, 수만 FF 로 합성)
                        //  pfw==0 은 라이브 버스 사용+래치, pfw>0 은 래치본(pf_word) 사용.
                        word_src = (pfw==0) ? i_gfb_rdata : pf_word;
                        if(pfw==0) pf_word <= i_gfb_rdata;
                        ff_tmp = {GFB_WIDTH{1'b0}};
                        for(ps=0;ps<8;ps=ps+1)
                            if(ps<in_folds)
                                ff_tmp[ps*128 +: 128] = word_src[((((pf_p+pfw)*in_folds+ps)%16))*128 +: 128];
                        if(pfw < px_per_word && (pf_p+pfw) < pf_limit)
                            ff_mem[(ff_wr+pfw)%FF_DEPTH] <= ff_tmp;   // 1 write/clk
                        if(pfw + 1 >= px_per_word) begin
                            // 워드 내 모든 픽셀 push 완료
                            ff_wr<= ff_wr + ((pf_p+px_per_word<=pf_limit) ? px_per_word : (pf_limit-pf_p));
                            pf_p <= pf_p + px_per_word;
                            pf_state<=PF_IDLE;
                        end else pfw <= pfw + 1;
                    end
                endcase
            end
        end
    end

    // --- supply (conv_block 공급 전담, 참고 TB 규약) ---
    //   픽셀당: valid 펄스 -> ready_out(c_rout) 대기 -> 폴딩간격(FOLD_WAIT) -> 다음.
    //   conv_block 은 픽셀 사이 채널폴딩 시간이 필요 -> 연속공급 금지.
    localparam SP_IDLE=0, SP_LOAD=1, SP_WAIT=2, SP_FOLD=3;
    reg [1:0] sp_state;
    reg [19:0] sp_cnt;        // 공급한 픽셀 수 (실+warmup)
    reg [3:0] sp_fold;        // 폴딩 대기 카운터
    localparam SP_FOLD_N = 3; // 폴딩 간격 (참고 TB stall_wait<3)
    // 속도 최적화: in_folds=1(단일 폴드, 채널폴딩 불필요)이면 폴딩대기 제거(3->1).
    //   값 불변(연산구조 동일, 픽셀간 대기만 단축). 64x64 체크포인트+AXI 로 불변 증명.
    wire [3:0] sp_fold_n_dyn = (in_folds <= 5'd1) ? 4'd1 : SP_FOLD_N[3:0];
    always @(posedge clk) begin
        if(!rst_n) begin
            cv3<=0; cpix3<=0; sp_cnt<=0; ff_rd<=0; sp_state<=SP_IDLE; sp_fold<=0;
        end else begin
            if(state==S3_GINIT) begin
                sp_cnt<=0; ff_rd<=0; cv3<=0; sp_state<=SP_LOAD; sp_fold<=0;
            end else if(1'b1) begin
                // ===== in_folds==1 스트리밍 fast-path (채널폴딩 불필요 -> 1 px/cycle) =====
                //   c_rout(conv ready) high 이거나 대기중 valid 없을 때 다음 픽셀 제시.
                //   c_rout low & cv3 high 면 hold(backpressure). LOAD/WAIT/FOLD 왕복 제거.
                if(c_rout || !cv3) begin
                    if(sp_cnt < in_pixels) begin
                        if(!ff_empty) begin
                            cpix3<=ff_mem[ff_rd[4:0]]; ff_rd<=ff_rd+1; cv3<=1; sp_cnt<=sp_cnt+1;
                        end else cv3<=0;   // FIFO 비면 버블(prefetch 채울때까지)
                    end else if(sp_cnt < in_pixels + feed_extra) begin
                        cpix3<={GFB_WIDTH{1'b0}}; cv3<=1; sp_cnt<=sp_cnt+1;  // drain warmup
                    end else cv3<=0;
                end
            end else begin
                case(sp_state)
                    SP_LOAD: begin
                        // 다음 픽셀 준비 (실픽셀 FIFO pop / warmup 0패딩)
                        //  공급 기준 = in_pixels (stride2 면 입력 전체). 출력은 collect 가 셈.
                        if(sp_cnt < in_pixels) begin
                            if(!ff_empty) begin
                                cpix3<=ff_mem[ff_rd[4:0]]; ff_rd<=ff_rd+1; cv3<=1; sp_state<=SP_WAIT;
                            end // FIFO 비면 대기 (prefetch 채울때까지)
                        end else if(sp_cnt < in_pixels + feed_extra) begin
                            cpix3<={GFB_WIDTH{1'b0}}; cv3<=1; sp_state<=SP_WAIT;
                        end // 공급 끝 -> SP_LOAD 머물며 종료(메인 FSM 이 처리)
                    end
                    SP_WAIT: begin
                        // conv_block 이 받을 때(c_rout=1)까지 valid 유지
                        if(c_rout) begin cv3<=0; sp_cnt<=sp_cnt+1; sp_fold<=0; sp_state<=SP_FOLD; end
                    end
                    SP_FOLD: begin
                        // 폴딩 간격 대기 (ready_in=1 인 클럭 카운트)
                        if(sp_fold >= sp_fold_n_dyn-1) sp_state<=SP_LOAD;
                        else sp_fold<=sp_fold+1;
                    end
                endcase
            end
        end
    end

    // --- collect (출력 수집 전담, 빽빽 패킹) ---
    reg [19:0] pp_cnt3; reg s3_pp_done;
    reg s3_we; reg [13:0] s3_waddr; reg [GFB_WIDTH-1:0] s3_wdata;
    reg [4:0] s3_last_slot; reg [13:0] s3_last_addr; reg [GFB_WIDTH-1:0] s3_last_word;
    reg [19:0] gfold3; reg [4:0] slot3; reg [GFB_WIDTH-1:0] ow3;
    // RMW: 다그룹 빽빽패킹 시, 한 워드에 여러 그룹 슬롯이 섞임.
    //   그룹 순차처리라 그룹0 이 짝수슬롯, 그룹1 이 홀수슬롯 등.
    //   -> 워드 완성(또는 워드 전환) 시 grp>0 이면 기존워드 read 후 병합 write.
    //   collect 전용 read(c_rmw_re/raddr) 사용, GFB read mux 에 연결.
    //   2단계: 워드쓸준비됨 -> RMW_READ(read요청) -> RMW_MERGE(병합+write)
    reg c_rmw_re; reg [13:0] c_rmw_raddr;
    reg [1:0] cstate; localparam C_IDLE=0, C_RMWR=1, C_MERGE=2;
    reg rmw_stall;  // 워드전환 감지 즉시 set, RMW 끝나면 clear
    wire rmw_busy = (cstate != C_IDLE) | rmw_stall;
    assign o1_busy = (o1_state != O1_IDLE);  // 1x1 collect RMW 중 backpressure
    reg [13:0] pend_addr; reg [GFB_WIDTH-1:0] pend_word; reg [15:0] pend_mask; // 채운 슬롯 마스크
    reg [GFB_WIDTH-1:0] merge_word; integer mi;
    reg [13:0] cur_waddr; reg [15:0] pend_mask_rmw; reg [13:0] rmw_addr;
    reg [GFB_WIDTH-1:0] hold_pix; reg [4:0] hold_slot; reg [13:0] hold_waddr; reg hold_valid;
    // [3x3 collect 는 data_writer 모듈로 분리됨 - 위 u_dw 인스턴스]

    // ===== 메인 FSM =====
    reg [19:0] feed_p;        // 타일 내 공급 픽셀 (0..tile_n-1)
    reg [19:0] pp_cnt;        // 타일 내 수집 픽셀
    reg [13:0] rd_word;       // 타일 적재용 워드 카운터
    reg [9:0]  rd_slot_pix;   // 적재중 픽셀
    reg [3:0]  flush_cnt;
    reg [19:0] global_pp;
    reg [19:0] gfold_o; reg [4:0] slot_o;
    // [누산화] 출력/입력 주소 곱셈 제거용.
    //   gfold_o=(tile_base+pp_cnt)*out_grps+grp.  tile_base 는 마지막 타일을 제외하면
    //   항상 TILE_PIX(=64=2^6) 씩 진행하므로(부분타일은 곧장 S_DONE) 곱셈이 시프트/가산으로 환원.
    //   grp0_base = (tile_base)*out_grps (그룹0 픽셀0 의 gfold).
    //   gfold_o_acc = 현재 출력픽셀 gfold (pp_cnt 증가마다 +out_grps).
    //   tbase_in_words = (tile_base*in_folds)/16 (입력 워드 기준주소).
    reg [19:0] grp0_base, gfold_o_acc, tbase_in_words;
    wire [19:0] tile_step_o = {out_grps,6'b0};  // TILE_PIX(64)*out_grps = out_grps<<6
    wire [19:0] tile_step_i = {in_folds,2'b0};   // (64*in_folds)/16 = 4*in_folds = in_folds<<2
    reg [GFB_WIDTH-1:0] out_word_n;
    reg [4:0] last_slot; reg [13:0] last_word_addr; reg [GFB_WIDTH-1:0] last_word;
    // 1x1 출력 RMW (다그룹 빽빽패킹)
    reg [15:0] o1_mask; reg [13:0] o1_addr, o1_cur_addr, o1_hold_addr;
    reg [GFB_WIDTH-1:0] o1_pend, o1_merge; reg [GFB_WIDTH-1:0] o1_hold_pix; reg [4:0] o1_hold_slot;
    integer o1_i;

    // 픽셀 추출: 타일버퍼는 이미 2048b 픽셀로 저장됨 -> tilebuf[feed_p] 직접 사용
    // 타일 적재: GFB 워드를 읽어, 워드 내 각 픽셀(INF슬롯)을 tilebuf 에 픽셀로 재구성.
    //   단순화: GFB 빽빽워드 1개 = 16/INF 픽셀. 워드 읽어 픽셀 분해 저장.
    integer s;
    reg [GFB_WIDTH-1:0] rdw;
    reg [5:0] pix_in_word;    // 워드당 픽셀수(최대 16) - 4비트로는 16 표현불가    // 워드당 픽셀 수 = 16/INF
    reg [5:0] tpw;            // S_TILE_WR: 워드내 픽셀 직렬 적재 카운터
    reg [9:0] tload;          // 타일에 적재된 픽셀 수

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE; o_done<=0; o_busy<=0; grp<=0; grp_reset<=0;
            m_re<=0; m_we<=0; o_req_grp<=0; out_ready<=1;
            feed_p<=0; pp_cnt<=0; c_val<=0; c_pix<=0; flush_cnt<=0;
            tile_base<=0; tile_n<=0; rd_word<=0; tload<=0; global_pp<=0;
            o1_state<=0; o1_mask<=0; dw_start<=0;
            out_word<=0; out_word_addr<=0;
            du_start<=0; stem_flush<=0; stem_flush_valid<=0; flush_p<=0;
        end else begin
            o_done<=0; grp_reset<=0; m_re<=0; m_we<=0;
            case(state)
                S_IDLE: if(i_start) begin
                    o_busy<=1; tile_base<=0; global_pp<=0;
                    rd_word<=0; tload<=0; out_word<=0;
                    px_per_word <= 16/in_folds;        // [TIMING] 1회 계산(상수)
                    grp0_base<=0; tbase_in_words<=0;   // [누산화] 타일 기준주소 초기화
                    if(i_is_layer_0) begin grp<=0; state<=ST_GINIT; end
                    else if(i_is_1x1) state<=S_TILE_RD;
                    else begin grp<=0; state<=S3_GINIT; end
                end
                // ===== 타일 입력 적재: GFB 워드 읽기 =====
                S_TILE_RD: begin
                    m_raddr <= i_in_base + tbase_in_words + rd_word;  // [누산화] (tile_base*in_folds)/16 제거
                    m_re<=1;
                    state<=S_TILE_LAT;
                end
                S_TILE_LAT: begin
                    // GFB read latency 1클럭 대기 (이 클럭에 rdata 도착)
                    state<=S_TILE_WAIT;
                end
                S_TILE_WAIT: begin
                    // GFB latency 1클럭 후 rdata. 워드를 래치하고, 픽셀 단위 직렬 적재로.
                    //  (기존: 한 클럭에 16슬롯 부분쓰기 -> 가변 비트오프셋 + 다중쓰기로
                    //   BRAM 추론 실패. 1 full-word/clk 직렬쓰기로 변경해 BRAM 추론 가능)
                    rdw <= i_gfb_rdata;
                    pix_in_word <= 16 / in_folds;   // 워드당 픽셀 수
                    tpw <= 0;
                    state <= S_TILE_WR;
                end
                S_TILE_WR: begin
                    // 픽셀 tpw 의 in_folds 슬롯을 bit0 정렬해 tilebuf[tload+tpw] 에 full-word 1개 적재.
                    //  rdw 에서 슬롯 (tpw*in_folds) 를 bit0 으로 우시프트 -> fold f 가 bit f*128.
                    //  상위(미사용) 비트는 후속 픽셀 데이터지만 conv_block 이 total_ch_in 만 읽어 무시.
                    if((tload + tpw) < TILE_PIX)
                        tilebuf[tload + tpw] <= rdw >> ((tpw*in_folds)*128);
                    if(tpw + 1 >= pix_in_word) begin
                        // 워드 내 모든 픽셀 적재 완료 -> 다음 워드/그룹처리
                        tload <= tload + pix_in_word;
                        if( (tload + pix_in_word >= TILE_PIX) ||
                            (tile_base + tload + pix_in_word >= i_target_pixel) ) begin
                            tile_n <= (tile_base + tload + pix_in_word >= i_target_pixel) ?
                                      (i_target_pixel - tile_base) : TILE_PIX;
                            grp<=0; state<=S_GRP_INIT;
                        end else begin
                            rd_word<=rd_word+1; state<=S_TILE_RD;
                        end
                    end else begin
                        tpw <= tpw + 1;
                    end
                end
                // ===== 그룹 루프 =====
                S_GRP_INIT: begin
                    grp_reset<=1; o_req_grp<=grp;
                    feed_p<=0; pp_cnt<=0; c_val<=0;
                    out_word<=0; o1_mask<=0; o1_state<=O1_IDLE;  // 그룹별 누적 초기화
                    gfold_o_acc<=grp0_base+grp;   // [누산화] 그룹 첫 픽셀(pp_cnt=0) gfold
                    state<=S_FEED;
                end
                S_FEED: begin
                    // 공급 (hold-until-accept). 3x3 은 warmup 위해 tile_n+feed_extra 까지,
                    //   tile_n 초과분은 0패딩. 1x1 은 feed_extra=0.
                    if(c_val && c_rout) begin
                        if(feed_p+1 < tile_n + feed_extra) begin
                            feed_p<=feed_p+1; c_val<=1;
                            c_pix<= (feed_p+1 < tile_n) ? tilebuf[feed_p+1] : {GFB_WIDTH{1'b0}};
                        end else c_val<=0;
                    end else if(!c_val && feed_p < tile_n + feed_extra) begin
                        c_val<=1; c_pix<= (feed_p < tile_n) ? tilebuf[feed_p] : {GFB_WIDTH{1'b0}};
                    end
                    // 수집: 빽빽 패킹 출력, 워드전환 시 직전워드 write/RMW (다그룹 지원)
                    //   grp==0: 부분워드 write. grp>0: 기존워드 RMW 병합.
                    // grp_reset 펄스 사이클에는 post_processor 리셋이 아직 반영 전이라
                    //   직전 레이어/그룹의 stale pp_vout 가 남아 유령 픽셀로 수집되는 문제 방지.
                    if(pp_vout && out_ready && o1_state==O1_IDLE && !grp_reset) begin
                        gfold_o = gfold_o_acc;   // [누산화] (tile_base+pp_cnt)*out_grps+grp 제거
                        slot_o  = gfold_o % 16;
                        o1_cur_addr = i_out_base + (gfold_o/16);
                        if(o1_mask!=0 && o1_cur_addr!=o1_addr) begin
                            // 워드 전환: 직전워드 확정
                            if(grp==0) begin
                                m_we<=1; m_waddr<=o1_addr; m_wdata<=out_word;
                                out_word_n=0; out_word_n[slot_o*128 +: 128]=pp_pix; out_word<=out_word_n;
                                o1_mask<=(16'd1<<slot_o); o1_addr<=o1_cur_addr;
                                pp_cnt<=pp_cnt+1; gfold_o_acc<=gfold_o_acc+out_grps;
                                if(pp_cnt+1>=tile_n) begin flush_cnt<=0; state<=S_FLUSH; end
                            end else begin
                                // RMW: 직전워드 read 요청
                                o1_pend<=out_word; m_re<=1; m_raddr<=o1_addr; o1_state<=O1_RMWR;
                                o1_hold_pix<=pp_pix; o1_hold_slot<=slot_o; o1_hold_addr<=o1_cur_addr;
                            end
                        end else begin
                            out_word_n = out_word; out_word_n[slot_o*128 +: 128] = pp_pix;
                            out_word <= out_word_n; o1_mask<=o1_mask|(16'd1<<slot_o); o1_addr<=o1_cur_addr;
                            pp_cnt<=pp_cnt+1; gfold_o_acc<=gfold_o_acc+out_grps;
                            if(pp_cnt+1>=tile_n) begin flush_cnt<=0; state<=S_FLUSH; end
                        end
                    end else if(o1_state==O1_RMWR) begin
                        o1_state<=O1_MERGE;  // GFB read 1clk 대기
                    end else if(o1_state==O1_MERGE) begin
                        o1_merge = i_gfb_rdata;
                        for(o1_i=0;o1_i<16;o1_i=o1_i+1)
                            if(o1_mask[o1_i]) o1_merge[o1_i*128 +: 128]=o1_pend[o1_i*128 +: 128];
                        m_we<=1; m_waddr<=o1_addr; m_wdata<=o1_merge;
                        // hold 픽셀 새 워드에
                        out_word_n=0; out_word_n[o1_hold_slot*128 +: 128]=o1_hold_pix; out_word<=out_word_n;
                        o1_mask<=(16'd1<<o1_hold_slot); o1_addr<=o1_hold_addr; o1_state<=O1_IDLE;
                        pp_cnt<=pp_cnt+1; gfold_o_acc<=gfold_o_acc+out_grps;
                        if(pp_cnt+1>=tile_n) begin flush_cnt<=0; state<=S_FLUSH; end
                    end
                end
                S_FLUSH: begin
                    // 마지막 워드(o1_mask 잔여) 확정
                    if(flush_cnt==0 && o1_mask!=0 && o1_state==O1_IDLE) begin
                        if(grp==0) begin
                            m_we<=1; m_waddr<=o1_addr; m_wdata<=out_word; o1_mask<=0;
                        end else begin
                            o1_pend<=out_word; m_re<=1; m_raddr<=o1_addr; o1_state<=O1_FRMW;
                        end
                    end else if(o1_state==O1_FRMW) begin
                        o1_state<=O1_FMERGE;
                    end else if(o1_state==O1_FMERGE) begin
                        o1_merge=i_gfb_rdata;
                        for(o1_i=0;o1_i<16;o1_i=o1_i+1)
                            if(o1_mask[o1_i]) o1_merge[o1_i*128 +: 128]=o1_pend[o1_i*128 +: 128];
                        m_we<=1; m_waddr<=o1_addr; m_wdata<=o1_merge; o1_mask<=0; o1_state<=O1_IDLE;
                    end
                    if(flush_cnt>=4) begin
                        if(grp+1 < out_grps) begin grp<=grp+1; state<=S_GRP_INIT; end
                        else begin
                            if(tile_base + tile_n >= i_target_pixel) state<=S_DONE;
                            else begin
                                tile_base<=tile_base+tile_n; rd_word<=0; tload<=0;
                                // [누산화] 다음 타일 기준주소(이 분기는 항상 full tile=TILE_PIX)
                                grp0_base<=grp0_base+tile_step_o;
                                tbase_in_words<=tbase_in_words+tile_step_i;
                                out_word<=0; o1_mask<=0; state<=S_TILE_RD;
                            end
                        end
                    end else flush_cnt<=flush_cnt+1;
                end
                // ===== 3x3 행 스트리밍 (그룹 제어만; read/supply 는 별도 always) =====
                S3_GINIT: begin
                    grp_reset<=1; o_req_grp<=grp;
                    dw_start<=1;  // data_writer 그룹 시작 (o_grp_done 클리어)
                    state<=S3_RUN;
                end
                S3_RUN: begin
                    dw_start<=0;
                    // dw_start 직후 1클럭은 grp_done 클리어 대기 (이전 그룹 done 무시).
                    // dw_start=0 이고 dw_grp_done=1 일때만 진짜 이 그룹 완료.
                    if(!dw_start && dw_grp_done) begin flush_cnt<=0; state<=S3_FLUSH; end
                end
                S3_FLUSH: begin
                    // data_writer 가 마지막워드까지 완전히 write 완료(dw_flush_done) 후 진행
                    if(dw_flush_done && flush_cnt>=2) begin
                        if(grp+1<out_grps) begin grp<=grp+1; state<=S3_GINIT; end
                        else state<=S_DONE;
                    end else flush_cnt<=flush_cnt+1;
                end
                // ===== stem 6x6 (wrapper_6x6 공급, collect 재사용) =====
                ST_GINIT: begin
                    grp_reset<=1; o_req_grp<=0;
                    dw_start<=1;  // data_writer 그룹 시작 (stem 은 1그룹)
                    du_start<=1; stem_flush<=0; stem_flush_valid<=0; flush_p<=0; state<=ST_RD;
                end
                ST_RD: begin
                    grp_reset<=0; dw_start<=0; du_start<=0;
                    state<=ST_FEED;
                end
                ST_LAT: state<=ST_FEED;
                ST_FEED: begin
                    // 실픽셀: unpacker 가 stem_pix24/stem_pvalid(조합 mux)로 직접 공급.
                    // unpacker 완료(du_done) -> flush 단계(라인버퍼 드레인용 0 공급).
                    if(du_done && !stem_flush) begin
                        stem_flush<=1; flush_p<=0; stem_flush_valid<=1;
                    end else if(stem_flush) begin
                        if(stem_flush_valid && stem_wrdy) begin
                            if(flush_p+1 < 3*i_img_width + 16) flush_p<=flush_p+1;
                            else stem_flush_valid<=0;
                        end
                    end
                    // 종료 체크
                    if(stem_flush && flush_p >= 3*i_img_width + 15 && !dw_start && dw_grp_done) begin
                        flush_cnt<=0; state<=ST_FLUSH;
                    end
                end
                ST_FLUSH: begin
                    if(flush_cnt>=4) state<=S_DONE;
                    else flush_cnt<=flush_cnt+1;
                end
                S_DONE: begin o_done<=1; o_busy<=0; state<=S_IDLE; end
            endcase
        end
    end
    // GFB 포트 mux (1x1=메인 m_*, 3x3=prefetch re3/raddr3 + collect s3_*)
    assign o_gfb_re    = dw_rmw_re ? 1'b1 : (i_is_layer_0 ? du_re : (i_is_1x1 ? m_re : re3));
    assign o_gfb_raddr = dw_rmw_re ? dw_rmw_raddr : (i_is_layer_0 ? du_raddr : (i_is_1x1 ? m_raddr : raddr3));
    assign o_gfb_we    = i_is_1x1 ? m_we    : dw_we;
    assign o_gfb_waddr = i_is_1x1 ? m_waddr : dw_waddr;
    assign o_gfb_wdata = i_is_1x1 ? m_wdata : dw_wdata;
endmodule
