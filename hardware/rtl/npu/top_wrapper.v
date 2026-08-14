`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: top_wrapper (배선 전담 - 패턴 A)
//
// 역할: 모든 엔진 + 공용자원 + sequencer_fsm 을 인스턴스화하고 배선만.
//   제어는 sequencer_fsm 이 전담. 한 번에 1개 엔진만 active -> GFB 포트 mux.
//
// 계층:
//   top_wrapper
//    ├─ sequencer_fsm (제어)
//    ├─ conv_engine / pool_engine / vec_engine / head_engine (각 내부 FSM)
//    ├─ parameter_rom_array / weight_rom_array / param_rom_array (공용 ROM)
//    └─ GFB 는 외부 포트 (TB/실제 BRAM)
//
// GFB 포트 mux: active_engine 에 따라 raddr/re/waddr/we/wdata 선택.
//   i_gfb_rdata 는 모든 엔진에 broadcast.
//////////////////////////////////////////////////////////////////////////////////
module top_wrapper #(
    parameter DATA_WIDTH=8, parameter T0=16, parameter FOLD_CH=16,
    parameter GFB_WIDTH=2048, parameter ADDR_WIDTH=14, parameter NUM_STEPS=53,
    parameter [7:0] NMS_THRESH  = 8'd0
)(
    input  wire clk, rst_n,
    input  wire i_start,
    output wire o_all_done,
    // GFB (외부 - TB 모델 또는 실제 BRAM)
    output wire [ADDR_WIDTH-1:0] o_gfb_raddr,
    output wire        o_gfb_re,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,
    input  wire        i_gfb_rvalid,
    output wire [ADDR_WIDTH-1:0] o_gfb_waddr,
    output wire        o_gfb_we,
    output wire [GFB_WIDTH-1:0]  o_gfb_wdata,
    // Head 96-bit AXI 출력
    output wire [95:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);
    localparam CONV=0, POOL=1, VEC=2, HEAD=3;

    // ===== sequencer_fsm <-> parameter_rom =====
    wire [6:0] layer_id; wire prom_en;
    wire [1:0] s_stride; wire s_isdet; wire s_isp4; wire [9:0] s_tch,s_outch; wire [19:0] s_tgt;
    wire [13:0] s_rA,s_rB,s_wa,s_waddr_w; wire [9:0] s_pa; wire [10:0] s_imgw;
    wire s_isup,s_iscc,s_isbyp,s_ispool,s_islayer0,s_is1x1,s_silu;
    // img_width 의미: stride2/stem 이면 입력너비(window_generator 요구), 아니면 출력너비.
    //   stride2/stem: 입력 정사각 -> 입력높이=입력너비=s_imgw. src_pixel=s_imgw^2.
    //   일반(stride1): 출력 -> img_height=ceil(tgt/imgw). src_pixel 불필요(0).
    wire s_in_coord = s_islayer0 | (s_stride==2'd2);   // img_width 가 입력너비인 경우
    wire [10:0] s_imgh = s_imgw;
    wire [19:0] s_srcpix = s_in_coord ? (s_imgw*s_imgw) : 20'd0;

    parameter_rom_array #(.ROM_DEPTH(NUM_STEPS)) u_prom(
        .clk(clk),.en(prom_en),.layer_id(layer_id),
        .stride_out(s_stride),.is_detect_out(s_isdet),.is_p4_out(s_isp4),.total_ch_in_out(s_tch),.out_ch_out(s_outch),
        .target_pixel_out(s_tgt),.raddr_A_out(s_rA),.raddr_B_out(s_rB),.waddr_out(s_wa),
        .weight_addr_out(s_waddr_w),.param_addr_out(s_pa),.img_width_out(s_imgw),
        .is_upsample_out(s_isup),.is_concat_out(s_iscc),.is_byp_out(s_isbyp),.is_pool_out(s_ispool),
        .is_layer_0_out(s_islayer0),.is_1x1_out(s_is1x1),.silu_en_out(s_silu),.in_place_out());

    // ===== sequencer_fsm =====
    wire [1:0] active;
    wire conv_start,pool_start,vec_start,head_start;
    wire conv_done,pool_done,vec_done,head_done;
    wire [4:0] conv_req_grp, conv_req_fold;
    wire [13:0] seq_waddr; wire [9:0] seq_paddr; wire is_p4;

    sequencer_fsm #(.NUM_STEPS(NUM_STEPS),.ADDR_WIDTH(ADDR_WIDTH)) u_fsm(
        .clk(clk),.rst_n(rst_n),.i_start(i_start),.o_all_done(o_all_done),
        .o_layer_id(layer_id),.o_prom_en(prom_en),
        .i_is_detect(s_isdet),.i_is_upsample(s_isup),.i_is_concat(s_iscc),
        .i_is_byp(s_isbyp),.i_is_pool(s_ispool),.i_is_1x1(s_is1x1),
        .i_total_ch_in(s_tch),.i_out_ch(s_outch),
        .i_weight_addr(s_waddr_w),.i_raddr_A(s_rA),.i_param_addr(s_pa),
        .o_active(active),
        .o_conv_start(conv_start),.o_pool_start(pool_start),.o_vec_start(vec_start),.o_head_start(head_start),
        .i_conv_done(conv_done),.i_pool_done(pool_done),.i_vec_done(vec_done),.i_head_done(head_done),
        .i_conv_req_grp(conv_req_grp),.i_conv_req_fold(conv_req_fold),
        .o_weight_addr(seq_waddr),.o_param_addr(seq_paddr),.o_is_p4(is_p4));

    // ===== 공용 weight/param ROM =====
    wire [2047:0] w1; wire [18431:0] w3;
    wire [255:0] bias1,M01,bias3,M03; wire [127:0] n1,n3;
    weight_rom_array u_wrom(.clk(clk),.ready_in(1'b1),.waddr(seq_waddr),
        .weight_3x3_out(w3),.weight_1x1_out(w1));
    param_rom_array u_prom2(.clk(clk),.ready_in(1'b1),.paddr(seq_paddr),
        .bias_1x1_out(bias1),.M0_1x1_out(M01),.n_1x1_out(n1),
        .bias_3x3_out(bias3),.M0_3x3_out(M03),.n_3x3_out(n3));
    // conv 가 1x1/3x3 에 따라 bias/M0/n 선택
    wire [255:0] c_bias = s_is1x1? bias1: bias3;
    wire [255:0] c_M0   = s_is1x1? M01  : M03;
    wire [127:0] c_n    = s_is1x1? n1   : n3;
    // n 재패킹: ROM 은 8bit/ch (128b), conv 는 5bit/ch (80b)
    wire [T0*5-1:0] c_n5;
    genvar gn;
    generate for(gn=0;gn<T0;gn=gn+1) begin: g_n5
        assign c_n5[gn*5 +: 5] = c_n[gn*8 +: 5];
    end endgenerate

    // ===== conv_engine =====
    wire [13:0] conv_ra,conv_wa; wire conv_re,conv_we; wire [2047:0] conv_wd;
    conv_engine #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.TILE_PIX(64),.WLAT(1),.PLAT(0)) u_conv(
        .clk(clk),.rst_n(rst_n),.i_start(conv_start),.o_done(conv_done),.o_busy(),
        .i_in_base(s_rA),.i_out_base(s_wa),.i_total_ch_in(s_tch),.i_out_ch(s_outch),
        .i_target_pixel(s_tgt),.i_img_width(s_imgw),.i_img_height(s_imgh),
        .i_stride(s_stride),.i_is_1x1(s_is1x1),.i_is_layer_0(s_islayer0),.i_src_pixel(s_srcpix),.i_silu_en(s_silu),
        .o_req_grp(conv_req_grp),.o_req_fold(conv_req_fold),
        .i_weight(w1),.i_weight3(w3),.i_bias(c_bias),.i_M0(c_M0),.i_n(c_n5),
        .o_gfb_raddr(conv_ra),.o_gfb_re(conv_re),.i_gfb_rdata(i_gfb_rdata),
        .o_gfb_waddr(conv_wa),.o_gfb_we(conv_we),.o_gfb_wdata(conv_wd));

    // ===== pool_engine =====
    wire [13:0] pool_ra,pool_wa; wire pool_re,pool_we; wire [2047:0] pool_wd;
    pool_engine #(.MAX_CH(64),.MAX_WIDTH(40),.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH)) u_pool(
        .clk(clk),.rst_n(rst_n),.i_start(pool_start),.o_done(pool_done),.o_busy(),
        .i_in_base(s_rA),.i_out_base(s_wa),.i_total_ch_in(s_tch),
        .i_target_pixel(s_tgt),.i_img_width(s_imgw),.i_img_height(s_imgh),
        .o_gfb_raddr(pool_ra),.o_gfb_re(pool_re),.i_gfb_rdata(i_gfb_rdata),
        .o_gfb_waddr(pool_wa),.o_gfb_we(pool_we),.o_gfb_wdata(pool_wd));

    // ===== vec_engine =====
    wire [13:0] vec_ra,vec_wa; wire vec_re,vec_we; wire [2047:0] vec_wd;
    vec_engine #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH)) u_vec(
        .clk(clk),.rst_n(rst_n),.i_start(vec_start),.o_done(vec_done),.o_busy(),
        .i_is_concat(s_iscc),.i_is_byp(s_isbyp),.i_is_upsample(s_isup),
        .i_raddr_A(s_rA),.i_raddr_B(s_rB),.i_waddr(s_wa),
        .i_ch_a(s_tch),.i_ch_b(s_outch),.i_target_pixel(s_tgt),.i_img_width(s_imgw),
        .o_gfb_raddr(vec_ra),.o_gfb_re(vec_re),.i_gfb_rdata(i_gfb_rdata),
        .o_gfb_waddr(vec_wa),.o_gfb_we(vec_we),.o_gfb_wdata(vec_wd));

    // ===== head_engine =====
    wire [13:0] head_ra; wire head_re;
    head_engine #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.NMS_THRESH(NMS_THRESH)) u_head(
        .clk(clk),.rst_n(rst_n),.i_start(head_start),.o_done(head_done),.o_busy(),
        .i_raddr_A(s_rA),.i_img_width(s_imgw),.i_img_height(s_imgh),.i_is_p4(s_isp4),
        .o_gfb_re(head_re),.o_gfb_raddr(head_ra),.i_gfb_rdata(i_gfb_rdata),.i_gfb_rvalid(i_gfb_rvalid),
        .m_axis_tdata(m_axis_tdata),.m_axis_tvalid(m_axis_tvalid),.m_axis_tready(m_axis_tready));

    // ===== GFB 포트 mux (active_engine) =====
    assign o_gfb_raddr = (active==CONV)? conv_ra : (active==POOL)? pool_ra :
                         (active==VEC)?  vec_ra  : head_ra;
    assign o_gfb_re    = (active==CONV)? conv_re : (active==POOL)? pool_re :
                         (active==VEC)?  vec_re  : head_re;
    assign o_gfb_waddr = (active==CONV)? conv_wa : (active==POOL)? pool_wa : vec_wa;
    assign o_gfb_we    = (active==CONV)? conv_we : (active==POOL)? pool_we :
                         (active==VEC)?  vec_we  : 1'b0;  // head 는 write 없음
    assign o_gfb_wdata = (active==CONV)? conv_wd : (active==POOL)? pool_wd : vec_wd;
endmodule
