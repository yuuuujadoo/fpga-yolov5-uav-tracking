`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: npu_top (최종 Top wrapper - GFB 내부 인스턴스화 + 연산코어 + I/O 경로)
//
// 계층:
//   npu_top (시스템)
//    ├─ top_wrapper (연산 코어: sequencer_fsm + 4엔진 + 3 ROM)
//    ├─ global_feature_buffer (GFB) <- 내부 인스턴스화 (BRAM)
//    └─ GFB write mux: 이미지 적재(외부) vs 엔진 출력(코어)
//
// 동작 모드:
//   - LOAD 모드 (i_load_en): 외부에서 이미지를 GFB 에 적재 (i_ld_waddr/wdata/we)
//   - RUN 모드 (i_start): 코어가 추론. 엔진이 GFB read/write.
//
// 외부 포트 (실제 FPGA 는 여기에 DDR3/이더넷 IP 연결):
//   - i_load_*: 이미지 적재 (실제는 DDR3->DMA)
//   - i_start/o_all_done: 추론 제어
//   - m_axis_*: 96-bit BBox 출력 (실제는 이더넷)
//   - o_gfb_dbg_*: 디버그용 GFB read (TB 가 중간결과 확인)
//////////////////////////////////////////////////////////////////////////////////
module npu_top #(
    parameter GFB_WIDTH = 2048,
    parameter ADDR_WIDTH = 14,
    parameter GFB_DEPTH = 9600,
    parameter NUM_STEPS = 57,
    parameter [7:0] NMS_THRESH  = 8'd13  // 1차 NMS conf 임계값(Q0.7). 운용=13, 자가검증=0
)(
    input  wire clk, rst_n,
    // 이미지 적재 (LOAD 모드)
    input  wire        i_load_en,         // 1=적재모드 (GFB write 를 외부가 점유)
    input  wire [ADDR_WIDTH-1:0] i_ld_waddr,
    input  wire        i_ld_we,
    input  wire [GFB_WIDTH-1:0]  i_ld_wdata,
    // 추론 제어 (RUN 모드)
    input  wire        i_start,
    output wire        o_all_done,
    // BBox 출력
    output wire [95:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);
    // ===== 코어 <-> GFB 신호 =====
    wire [ADDR_WIDTH-1:0] core_raddr, core_waddr;
    wire core_re, core_we;
    wire [GFB_WIDTH-1:0] core_wdata;
    wire [GFB_WIDTH-1:0] gfb_rdata;
    wire gfb_rvalid;  // 코어는 rvalid 기대 (현재 top_wrapper 포트)

    // ===== 연산 코어 (top_wrapper) =====
    top_wrapper #(.GFB_WIDTH(GFB_WIDTH),.ADDR_WIDTH(ADDR_WIDTH),.NUM_STEPS(NUM_STEPS),
                  .NMS_THRESH(NMS_THRESH)) u_core(
        .clk(clk),.rst_n(rst_n),.i_start(i_start),.o_all_done(o_all_done),
        .o_gfb_raddr(core_raddr),.o_gfb_re(core_re),
        .i_gfb_rdata(gfb_rdata),.i_gfb_rvalid(gfb_rvalid),
        .o_gfb_waddr(core_waddr),.o_gfb_we(core_we),.o_gfb_wdata(core_wdata),
        .m_axis_tdata(m_axis_tdata),.m_axis_tvalid(m_axis_tvalid),.m_axis_tready(m_axis_tready));

    // ===== GFB write mux: LOAD 모드면 외부 적재, RUN 모드면 코어 =====
    wire [ADDR_WIDTH-1:0] gfb_waddr = i_load_en ? i_ld_waddr : core_waddr;
    wire        gfb_we    = i_load_en ? i_ld_we    : core_we;
    wire [GFB_WIDTH-1:0]  gfb_wdata = i_load_en ? i_ld_wdata : core_wdata;
    // GFB read: 코어 직결 (디버그 포트 제거)
    wire [ADDR_WIDTH-1:0] gfb_raddr = core_raddr;
    wire        gfb_re    = core_re;

    // ===== GFB 내부 인스턴스화 =====
    global_feature_buffer #(.DATA_WIDTH(GFB_WIDTH),.DEPTH(GFB_DEPTH),.ADDR_WIDTH(ADDR_WIDTH)) u_gfb(
        .clk(clk),
        .wdata(gfb_wdata),.waddr(gfb_waddr),.we(gfb_we),
        .raddr(gfb_raddr),.re(gfb_re),.rdata(gfb_rdata));


    // GFB 는 1클럭 latency. 코어가 기대하는 rvalid 생성 (re 1클럭 뒤).
    reg rvalid_r;
    always @(posedge clk) begin
        if(!rst_n) rvalid_r<=0;
        else rvalid_r<= core_re;  // 코어 re 1클럭 뒤 valid
    end
    assign gfb_rvalid = rvalid_r;
endmodule
