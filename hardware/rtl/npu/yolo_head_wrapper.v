`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// =================================================================================
// [모듈명]: yolo_head_wrapper
// [역할]: 
// - YOLOv5n NPU의 탐지(Detection)를 담당하는 최상위 Head 모듈입니다.
// - Global Buffer에서 데이터를 읽어오는 Formatter부터, 최종 좌표를 계산하는 Decoder까지 
// 4개의 하위 모듈을 인스턴스화하고 전선(Wire)으로 연결합니다.
// - 6개로 쪼개져 있는 계산 결과를 96-bit AXI-Stream 규격으로 묶어(Concat) 이더넷 통신부로 보냅니다.
// - [재사용 구조] Top 모듈에서 P3/P4 해상도 및 앵커 파라미터를 입력받아 무한히 재사용됩니다.
// =================================================================================
//////////////////////////////////////////////////////////////////////////////////

module yolo_head_wrapper #(
    // [정적 파라미터 선언부] 하드웨어의 물리적 크기를 결정하는 상수들
    parameter DATA_WIDTH     = 8,      // 양자화된 데이터 비트 폭 (INT8)
    parameter CHANNELS       = 6,      // 1개 앵커당 필요한 변수 개수 (tx, ty, tw, th, conf, cls)
    parameter MAX_COORD_BITS = 8,      // 최대 해상도(80)의 좌표를 담기 위한 비트 폭
    parameter OUT_WIDTH      = 16,     // BBox Decoder를 거쳐 팽창된 최종 좌표 비트 폭
    parameter GB_BUS_WIDTH   = 2048,   // Global Buffer의 거대 데이터 버스 폭
    parameter ADDR_WIDTH     = 15      // Global Buffer 메모리 주소 폭
)(
    // [1. 시스템 공통 신호]
    input  wire                   clk,         // 시스템 메인 클럭
    input  wire                   rst_n,       // Active-Low 비동기 리셋

    // =====================================================================
    // [2. Top 모듈로부터 들어오는 런타임 제어 및 동적 파라미터 (P3/P4 스위칭용)]
    // 설명: Top의 Main FSM이 P3일 때는 80x80 관련 값을, P4일 때는 40x40 관련 값을 넣어줍니다.
    // =====================================================================
    input  wire                   i_start,     // Head 파이프라인 가동 시작 트리거
    output wire                   o_done,      // 현재 레이어(해상도 전체) 처리 완료를 Top에 알리는 신호

    input  wire [ADDR_WIDTH-1:0]  i_base_addr, // Global Buffer에서 읽기 시작할 베이스 주소
    input  wire [7:0]             i_grid_w,    // 처리할 특징맵 가로 크기 (P3: 80, P4: 40)
    input  wire [7:0]             i_grid_h,    // 처리할 특징맵 세로 크기 (P3: 80, P4: 40)
    input  wire [3:0]             i_downshift, // 스케일 복원용 시프트 값 (P3: 4, P4: 3)
    
    // 앵커 박스 크기 파라미터 (미리 정의된 K-means 앵커 크기)
    input  wire [OUT_WIDTH-1:0]   i_pw_0, i_ph_0, // 0번 앵커 너비/높이
    input  wire [OUT_WIDTH-1:0]   i_pw_1, i_ph_1, // 1번 앵커 너비/높이
    input  wire [OUT_WIDTH-1:0]   i_pw_2, i_ph_2, // 2번 앵커 너비/높이

    // =====================================================================
    // [3. Global Buffer(BRAM) 읽기 인터페이스]
    // =====================================================================
    output wire                   o_gb_ren,    // GFB 읽기 활성화 신호
    output wire [ADDR_WIDTH-1:0]  o_gb_raddr,  // GFB 읽기 요청 주소
    input  wire [GB_BUS_WIDTH-1:0]i_gb_rdata,  // 1클럭 뒤 도착하는 2048-bit 거대 데이터
    input  wire                   i_gb_rvalid, // 도착한 데이터가 유효함을 알리는 신호

    // =====================================================================
    // [4. AXI-Stream 규격 외부 출력 인터페이스 (이더넷 FIFO 연결용)]
    // =====================================================================
    output wire [95:0]            m_axis_tdata,  // 96-bit로 묶인(Concat) 최종 1개 BBox 데이터 패키지
    output wire                   m_axis_tvalid, // 현재 96-bit 데이터가 진짜임을 알리는 유효 신호
    input  wire                   m_axis_tready  // 이더넷 FIFO가 꽉 찼을 때 0이 되어 NPU를 멈추는 Stall 신호
);

    // =========================================================================
    // [내부 연결 전선(Wire) 선언 구역]
    // 설명: 4개의 하위 모듈들을 엮어주기 위한 핏줄(Wire)들입니다.
    // =========================================================================

    // 1. Formatter -> Splitter 연결 와이어
    wire        w_fmt_valid;       // 48-bit 뭉치 데이터 유효 신호
    wire [47:0] w_fmt_data;        // 발골된 48비트(6채널) 알맹이 데이터
    wire [7:0]  w_fmt_cx, w_fmt_cy;// 화면상 X, Y 좌표
    wire [1:0]  w_fmt_anchor;      // 0,1,2번 앵커 식별자

    // 2. Splitter -> Sigmoid Array 연결 와이어
    wire        w_spl_valid;       // 8-bit로 찢어진 데이터 유효 신호
    wire [7:0]  w_spl_tx, w_spl_ty, w_spl_tw, w_spl_th, w_spl_conf, w_spl_cls; // 6가닥으로 찢어진 원시 데이터
    wire [7:0]  w_spl_cx, w_spl_cy;// Bypass 되는 좌표
    wire [1:0]  w_spl_anchor;      // Bypass 되는 앵커 식별자

    // 3. Sigmoid Array -> BBox Decoder 연결 와이어
    wire        w_sig_valid;       // Sigmoid를 통과한 데이터 유효 신호
    wire [7:0]  w_sig_tx, w_sig_ty, w_sig_tw, w_sig_th, w_sig_conf, w_sig_cls; // 0~127 스케일로 압축된 활성화 데이터
    wire [7:0]  w_sig_cx, w_sig_cy;// 타이밍을 맞춰 1클럭 지연된 좌표
    wire [1:0]  w_sig_anchor;      // 타이밍을 맞춰 1클럭 지연된 앵커 식별자

    // 4. BBox Decoder 출력 와이어 (최종 계산 결과)
    wire        w_dec_valid;       // 모든 연산이 끝난 최종 유효 신호
    wire signed [15:0] w_dec_bx, w_dec_by, w_dec_bw, w_dec_bh; // 실제 화면상의 16-bit 픽셀 좌표와 크기
    wire [7:0]  w_dec_conf, w_dec_cls; // 연산하지 않고 파이프라인 타이밍만 맞춘 신뢰도 및 확률


    // =========================================================================
    // [하위 모듈 1]: Data Formatter 인스턴스화
    // 역할: 2048비트를 읽어 32채널 패딩을 무시하고, 6채널(48-bit)씩 3번에 걸쳐 내보냅니다.
    // =========================================================================
    data_formatter #(
        .DATA_WIDTH(DATA_WIDTH), .CHANNELS(CHANNELS), .GB_BUS_WIDTH(GB_BUS_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_data_formatter (
        .clk(clk), .rst_n(rst_n),
        .i_start(i_start), .i_base_addr(i_base_addr), .i_grid_w(i_grid_w), .i_grid_h(i_grid_h),
        .o_done(o_done),
        .o_gb_ren(o_gb_ren), .o_gb_raddr(o_gb_raddr), .i_gb_rdata(i_gb_rdata), .i_gb_rvalid(i_gb_rvalid),
        .ready_in(m_axis_tready), // 💡외부 FIFO의 멈춤 신호를 가장 첫 모듈까지 라우팅(Backpressure)
        .o_split_valid(w_fmt_valid), .o_split_data(w_fmt_data), 
        .o_split_cx(w_fmt_cx), .o_split_cy(w_fmt_cy), .o_split_anchor(w_fmt_anchor)
    );

    // =========================================================================
    // [하위 모듈 2]: Data Splitter 인스턴스화
    // 역할: 48-bit 뭉치를 받아 각각 전용선(8-bit x 6가닥)으로 찢어줍니다.
    // =========================================================================
    data_splitter #(
        .DATA_WIDTH(DATA_WIDTH), .CHANNELS(CHANNELS), .MAX_COORD_BITS(MAX_COORD_BITS)
    ) u_data_splitter (
        .clk(clk), .rst_n(rst_n),
        .i_valid(w_fmt_valid), .i_data(w_fmt_data), 
        .i_cx(w_fmt_cx), .i_cy(w_fmt_cy), .i_anchor(w_fmt_anchor),
        .ready_in(m_axis_tready), // 💡Backpressure 라우팅
        .o_valid(w_spl_valid), 
        .o_tx(w_spl_tx), .o_ty(w_spl_ty), .o_tw(w_spl_tw), .o_th(w_spl_th), 
        .o_tconf(w_spl_conf), .o_tcls(w_spl_cls),
        .o_cx(w_spl_cx), .o_cy(w_spl_cy), .o_anchor(w_spl_anchor)
    );

    // =========================================================================
    // [하위 모듈 3]: Sigmoid Array 인스턴스화
    // 역할: 6개의 변수를 병렬로 Sigmoid LUT에 넣고, 우회(Bypass)하는 좌표들도 1클럭 지연시킵니다.
    // =========================================================================
    sigmoid_array #(
        .DATA_WIDTH(DATA_WIDTH), .MAX_COORD_BITS(MAX_COORD_BITS)
    ) u_sigmoid_array (
        .clk(clk), .rst_n(rst_n),
        .i_valid(w_spl_valid), .i_ready(m_axis_tready), // 💡Backpressure 라우팅
        .i_tx(w_spl_tx), .i_ty(w_spl_ty), .i_tw(w_spl_tw), .i_th(w_spl_th), 
        .i_tconf(w_spl_conf), .i_tcls(w_spl_cls),
        .i_cx(w_spl_cx), .i_cy(w_spl_cy), .i_anchor(w_spl_anchor),
        .o_valid(w_sig_valid),
        .o_tx(w_sig_tx), .o_ty(w_sig_ty), .o_tw(w_sig_tw), .o_th(w_sig_th), 
        .o_tconf(w_sig_conf), .o_tcls(w_sig_cls),
        .o_cx(w_sig_cx), .o_cy(w_sig_cy), .o_anchor(w_sig_anchor)
    );

    // =========================================================================
    // [하위 모듈 4]: BBox Decoder 인스턴스화
    // 역할: 최종 수학 공식(제곱, 곱셈, 시프트)을 적용하여 16-bit 픽셀 좌표로 복원합니다.
    // =========================================================================
    bbox_decoder #(
        .DATA_WIDTH(DATA_WIDTH), .COORD_BITS(MAX_COORD_BITS), .OUT_WIDTH(OUT_WIDTH)
    ) u_bbox_decoder (
        .clk(clk), .rst_n(rst_n),
        .i_valid(w_sig_valid), .ready_in(m_axis_tready), // 💡Backpressure 라우팅
        .i_tx(w_sig_tx), .i_ty(w_sig_ty), .i_tw(w_sig_tw), .i_th(w_sig_th), 
        .i_conf(w_sig_conf), .i_cls(w_sig_cls),
        .i_cx(w_sig_cx), .i_cy(w_sig_cy), .i_anchor(w_sig_anchor),
        // Top에서 받아온 동적 파라미터 쑤셔넣기
        .i_downshift(i_downshift), 
        .i_pw_0(i_pw_0), .i_ph_0(i_ph_0),
        .i_pw_1(i_pw_1), .i_ph_1(i_ph_1),
        .i_pw_2(i_pw_2), .i_ph_2(i_ph_2),
        // 최종 16/8비트 출력
        .o_valid(w_dec_valid),
        .o_bx(w_dec_bx), .o_by(w_dec_by), .o_bw(w_dec_bw), .o_bh(w_dec_bh),
        .o_conf(w_dec_conf), .o_cls(w_dec_cls)
    );


    // =========================================================================
    // [AXI-Stream 출력 패키징 (Concat) 구역]
    // 설명: 계산이 끝난 파편화된 데이터들을 96-bit 규격에 맞게 묶어서 출력 포트로 내보냅니다.
    // =========================================================================
    
    // 1. Zero-Padding (비트 폭 정렬)
    // bx, by, bw, bh는 이미 16-bit지만, conf와 cls는 8-bit입니다.
    // 메모리에 쓸 때 바이트(Byte) 정렬이 어긋나는 것을 막기 위해, 앞단에 0을 8개 붙여 16-bit로 억지 확장합니다.
    wire [15:0] w_padded_conf = {8'h00, w_dec_conf};
    wire [15:0] w_padded_cls  = {8'h00, w_dec_cls};

    // 2. Concatenation (전선 묶기)
    // 6개의 16-bit 덩어리를 중괄호 {} 연산자를 사용해 하나의 거대한 96-bit 버스로 묶어줍니다.
    // (보통 MSB 쪽에 확률을, LSB 쪽에 좌표를 두는 것이 일반적인 프로토콜 관례입니다)
    wire [95:0] w_combined_tdata = {w_padded_cls, w_padded_conf, w_dec_bh, w_dec_bw, w_dec_by, w_dec_bx};

    // 3. AXI-Stream 포트 매핑 (출력)
    // 묶어둔 96비트 데이터를 AXI-Stream 규격 이름인 m_axis_tdata에 바로 연결합니다.
    assign m_axis_tdata  = w_combined_tdata;
    // 디코더가 연산을 끝마치고 "유효하다"고 뿜어내는 신호를 m_axis_tvalid에 연결합니다.
    assign m_axis_tvalid = w_dec_valid;

    // ※ m_axis_tready 신호는 이미 위쪽의 인스턴스화 부분에서 모든 하위 모듈의 
    //    ready_in 포트로 브로드캐스팅(Broadcasting) 되어 완벽하게 연결되었습니다!

endmodule