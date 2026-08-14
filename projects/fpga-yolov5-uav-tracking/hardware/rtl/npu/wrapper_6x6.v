`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: wrapper_6x6
// Description: 
//   - Layer 0 전용 프론트엔드 모듈 (Line Buffer + Window Generator 통합)
//   - [검증용 Wrapper] 개별 모듈 합성 시 발생하는 Vivado의 BRAM 삭제(Pruning) 
//     현상을 원천 차단하여, 실제 칩에 탑재될 때의 정확한 자원 사용량(BRAM/LUT)을 측정함.
//   - [데이터플로우] 외부 픽셀 입력 -> 6줄 버퍼링 -> 6x6 윈도우 생성 -> Conv 코어로 전달
//   - [Handshake] 파이프라인 정지(Stall) 신호를 양방향으로 완벽하게 중계(Relay)함.
//////////////////////////////////////////////////////////////////////////////////

module wrapper_6x6 #(
    // -------------------------------------------------------------------------
    // [파라미터 선언] 하드웨어 아키텍처 스펙
    // -------------------------------------------------------------------------
    parameter DATA_WIDTH  = 24,  // RGB 3채널 * 8비트를 합친 24-bit 데이터 폭
    parameter MAX_WIDTH   = 640, // 시스템이 지원하는 이미지의 최대 가로 해상도
    parameter WINDOW_BITS = 864  // 6x6 윈도우 크기 (36픽셀 * 24비트 = 864비트)
)(
    // -------------------------------------------------------------------------
    // [입출력 포트 선언] 외부와 통신하는 핀들
    // -------------------------------------------------------------------------
    input  wire                   clk,          // 시스템 동기화의 기준이 되는 메인 클럭
    input  wire                   rst_n,        // 시스템을 초기화하는 비동기 리셋 플래그 (Active-Low)

    input  wire [10:0]            img_width,    // 현재 연산 중인 이미지의 가로 폭
    input  wire [10:0]            img_height,   // 현재 연산 중인 이미지의 세로 폭
    input  wire [1:0]             stride,       // 보폭 제어 신호 (현재 6x6은 내부 하드코딩되므로 래퍼에서만 받음)

    input  wire [DATA_WIDTH-1:0]  pixel_in,     // 외부 DMA나 카메라에서 주입되는 24-bit 픽셀 데이터
    input  wire                   valid_in,     // pixel_in 데이터가 유효함을 알리는 AXI-Stream TVALID 신호

    input  wire                   ready_in,     // 뒷단(MAC Array)에서 자신이 바쁘다며 멈추라고 보내는 TREADY 신호
    output wire                   ready_out,    // 앞단(DMA)으로 현재 이 모듈이 꽉 찼다고 알리는 TREADY 신호

    output wire [WINDOW_BITS-1:0] window_out,   // 조립이 완료되어 뒷단으로 방출되는 864-bit 거대 윈도우
    output wire                   valid_out     // window_out 데이터가 완벽한 정답임을 알리는 TVALID 신호
);

    // =========================================================================
    // [1] 배압 신호(Backpressure) 연결 와이어 선언
    // 설명: 파이프라인의 멈춤 신호가 꼬임 없이 역방향으로 잘 전달되도록 잇는 전선입니다.
    // =========================================================================
    wire lb_ready_out; // 라인버퍼가 앞단(DMA)으로 보내기 위해 내뱉는 ready 상태값
    wire wg_ready_out; // 윈도우 생성기가 라인버퍼를 멈추기 위해 내뱉는 ready 상태값

    // 💡 [다중 충돌 및 데드락 방어 로직] 
    // FSM의 강제 개입을 차단하고, 가장 맨 앞단인 라인 버퍼의 상태(lb_ready_out)를 
    // 모듈의 최종 ready_out으로 투명하게 직결(assign)합니다.
    assign ready_out = lb_ready_out;

    // =========================================================================
    // [2] 라인 버퍼 인스턴스화 (Line Buffer 6x6)
    // 설명: 1픽셀씩 들어오는 스트림을 받아 BRAM에 저장하고 6개의 줄(Row)로 동시에 내뿜습니다.
    // =========================================================================
    // 라인 버퍼에서 튀어나올 6줄의 24비트 데이터를 담을 관찰용 전선(wire)들입니다.
    wire [DATA_WIDTH-1:0] lb_row0, lb_row1, lb_row2, lb_row3, lb_row4, lb_row5;
    wire                  lb_valid_out; // 6줄 데이터가 유효함을 알리는 깃발

    line_buffer_6x6 #( 
        .DATA_WIDTH(DATA_WIDTH),    // 24비트 파라미터 전달
        .MAX_WIDTH(MAX_WIDTH)       // 640 파라미터 전달
    ) u_line_buffer (
        .clk(clk),                  // 메인 클럭 연결
        .rst_n(rst_n),              // 리셋 연결
        .img_width(img_width),      // 가로 폭 정보 연결
        
        .pixel_in(pixel_in),        // 외부에서 들어온 픽셀을 그대로 라인 버퍼에 주입
        .valid_in(valid_in),        // 외부 유효 깃발 주입
        
        .ready_in(wg_ready_out),    // 💡 뒷단인 윈도우 생성기의 멈춤 신호를 나의 브레이크로 수신
        
        // 버퍼링이 완료된 6줄의 데이터와 유효 깃발을 밖으로 뽑아냅니다.
        .row0_out(lb_row0), .row1_out(lb_row1), .row2_out(lb_row2),
        .row3_out(lb_row3), .row4_out(lb_row4), .row5_out(lb_row5),
        .valid_out(lb_valid_out), 
        
        .ready_out(lb_ready_out)    // 앞단으로 보낼 나의 바쁨 상태를 출력 (위에서 assign됨)
    );

    // =========================================================================
    // [3] 윈도우 생성기 인스턴스화 (Window Generator 6x6)
    // 설명: 라인 버퍼에서 나온 6개의 줄을 받아, 절대 좌표를 기반으로 6x6 윈도우로 조립합니다.
    // =========================================================================
    window_generator_6x6 #( 
        .DATA_WIDTH(DATA_WIDTH), 
        .MAX_WIDTH(MAX_WIDTH) 
    ) u_window_gen (
        .clk(clk), 
        .rst_n(rst_n), 
        .img_width(img_width), 
        .img_height(img_height), 
        
        // 💡 [핵심 버그 수정 구역]
        // window_generator_6x6.v 내부에서 Stride=2가 하드코딩되어 있으므로 
        // 포트 선언에 stride가 없습니다. 따라서 여기서 .stride() 맵핑을 완전히 지워야 합니다!
        
        // 라인 버퍼에서 쏟아져 나온 6줄의 데이터와 유효 깃발을 수신합니다.
        .row0_in(lb_row0), .row1_in(lb_row1), .row2_in(lb_row2), 
        .row3_in(lb_row3), .row4_in(lb_row4), .row5_in(lb_row5),
        .valid_in(lb_valid_out), 
        
        .ready_in(ready_in),        // 💡 가장 최종단(MAC Array)에서 들어온 멈춤 신호를 브레이크로 수신
        .ready_out(wg_ready_out),   // 내가 바쁠 때 라인 버퍼를 멈추기 위해 송신하는 상태값
        
        .window_out(window_out),    // 모듈 최종 출력 (864-bit 윈도우)
        .valid_out(valid_out)       // 모듈 최종 유효 깃발
    );

endmodule