`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: head_engine (Detect Head 후처리 엔진: P3/P4 BBox 디코드)
//
// 역할: GFB 의 헤드 conv 출력(18ch, 32패딩)을 읽어 sigmoid + bbox decode ->
//   96-bit BBox AXI 스트림 출력. yolo_head_wrapper(검증됨) 를 감싸는 얇은 래퍼.
//
// 다른 엔진과의 차이:
//   - 출력이 GFB write 가 아니라 96-bit AXI 스트림 (m_axis -> 이더넷 FIFO).
//   - 앵커(pw,ph) 는 모델 고정값(yaml) -> head_engine 내부 상수. grid 로 P3/P4 판별.
//
// P3/P4 시분할 재사용 (검증됨):
//   - 단일 yolo_head_wrapper 인스턴스를 P3, P4 로 2회 호출.
//   - P3: grid 80x80, downshift=4, anchors[10,13,16,30,33,23]
//   - P4: grid 40x40, downshift=3, anchors[30,61,62,45,59,119]
//   - is_p4 (또는 grid_w<=40) 로 판별해 앵커/downshift 선택.
//
// 명령어 비트맵 매핑:
//   i_base_addr = raddr_A (헤드 conv 출력 위치)
//   i_grid_w/h  = img_width/height
//   is_p4 판별 = (img_width <= 40)  또는 시퀀서가 별도 지정
//////////////////////////////////////////////////////////////////////////////////
module head_engine #(
    parameter GFB_WIDTH  = 2048,
    parameter ADDR_WIDTH = 14,       // Top GFB 주소폭
    parameter HEAD_ADDR  = 15,       // yolo_head_wrapper 주소폭
    parameter [7:0] NMS_THRESH = 8'd0)(
    input  wire clk, rst_n,
    input  wire        i_start,
    output wire        o_done,
    output wire        o_busy,
    // 디코드 필드
    input  wire [13:0] i_raddr_A,    // 헤드 출력 base
    input  wire [10:0] i_img_width,  // grid_w
    input  wire [10:0] i_img_height, // grid_h
    input  wire        i_is_p4,      // 0=P3, 1=P4 (시퀀서가 지정; 또는 grid 로 판별)
    // GFB read
    output wire        o_gfb_re,
    output wire [ADDR_WIDTH-1:0] o_gfb_raddr,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,
    input  wire        i_gfb_rvalid,
    // 96-bit BBox AXI 출력
    output wire [95:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);
    // ===== 앵커 상수 (best.pt 학습 앵커, auto-anchor) =====
    //   P3=[(10,5),(15,8),(30,14)], P4=[(71,36),(145,86),(347,218)]
    //   (yaml 기본값 아님 - 2-head 커스텀+UAV셋 auto-anchor 로 재학습된 픽셀앵커)
    wire [15:0] pw0 = i_is_p4 ? 16'd71  : 16'd10;
    wire [15:0] ph0 = i_is_p4 ? 16'd36  : 16'd5;
    wire [15:0] pw1 = i_is_p4 ? 16'd145 : 16'd15;
    wire [15:0] ph1 = i_is_p4 ? 16'd86  : 16'd8;
    wire [15:0] pw2 = i_is_p4 ? 16'd347 : 16'd30;
    wire [15:0] ph2 = i_is_p4 ? 16'd218 : 16'd14;
    wire [3:0]  downshift = i_is_p4 ? 4'd3 : 4'd4;

    // 주소폭 확장 (14 -> 15)
    wire [HEAD_ADDR-1:0] hw_raddr;
    assign o_gfb_raddr = hw_raddr[ADDR_WIDTH-1:0];
    assign o_busy = ~o_done;  // 간이 busy (wrapper 가 내부 상태 보유)

    // ===== head <-> NMS 내부 연결선 (선언을 사용보다 먼저: implicit net 방지) =====
    wire [95:0] hw_tdata; wire hw_tvalid, hw_tready;
    wire        frame_start = i_start & ~i_is_p4;   // P3 시작에만 index 리셋
    wire [15:0] nms_pass_cnt, nms_total_cnt;

    yolo_head_wrapper #(.GB_BUS_WIDTH(GFB_WIDTH),.ADDR_WIDTH(HEAD_ADDR)) u_head (
        .clk(clk),.rst_n(rst_n),.i_start(i_start),.o_done(o_done),
        .i_base_addr({{(HEAD_ADDR-14){1'b0}}, i_raddr_A}),
        .i_grid_w(i_img_width[7:0]),.i_grid_h(i_img_height[7:0]),.i_downshift(downshift),
        .i_pw_0(pw0),.i_ph_0(ph0),.i_pw_1(pw1),.i_ph_1(ph1),.i_pw_2(pw2),.i_ph_2(ph2),
        .o_gb_ren(o_gfb_re),.o_gb_raddr(hw_raddr),
        .i_gb_rdata(i_gfb_rdata),.i_gb_rvalid(i_gfb_rvalid),
        .m_axis_tdata(hw_tdata),.m_axis_tvalid(hw_tvalid),.m_axis_tready(hw_tready)
    );

    // ===== 1차 NMS (confidence prefilter) =====
    //   yolo_head_wrapper 96b 출력을 인라인 게이트. P3->P4 인덱스 연속 위해
    //   프레임 시작(=P3 head start)에만 리셋. backpressure 보존.
    nms_prefilter #(.IDX_WIDTH(16),.USE_CLS(0)) u_nms (
        .clk(clk),.rst_n(rst_n),
        .i_frame_start(frame_start),.i_thresh(NMS_THRESH),
        .i_tdata(hw_tdata),.i_tvalid(hw_tvalid),.i_tready(hw_tready),
        .o_tdata(m_axis_tdata),.o_tvalid(m_axis_tvalid),.o_tready(m_axis_tready),
        .o_pass_count(nms_pass_cnt),.o_total_count(nms_total_cnt)
    );
endmodule
