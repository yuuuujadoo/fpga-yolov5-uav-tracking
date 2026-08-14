`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: line_buffer_6x6
// Description: 
//   - Layer 0 (Stem Layer) 전용 초경량 프론트엔드 라인 버퍼
//   - [24-bit 구조] RGB 3채널 * 8비트 = 24-bit 데이터 패스 적용 (BRAM 소모 극소화)
//   - [5-Row Cascading] 5개의 BRAM을 폭포수처럼 연결하여 과거 5줄을 기억함
//   - [Stall 제어] ready_in 신호 연동으로 완벽한 파이프라인 Freeze 및 Backpressure 지원
//////////////////////////////////////////////////////////////////////////////////

module line_buffer_6x6 #(
    // -------------------------------------------------------------------------
    // [파라미터 선언] 하드웨어 아키텍처 스펙 정의
    // -------------------------------------------------------------------------
    parameter DATA_WIDTH = 24, // 8-bit * 3채널(RGB) = 24-bit 초경량 버스 적용
    parameter MAX_WIDTH  = 640 // 시스템이 지원하는 BRAM의 최대 가로 길이
)(
    // -------------------------------------------------------------------------
    // [입출력 포트 선언]
    // -------------------------------------------------------------------------
    input wire clk,                 // 시스템 메인 클럭 (모든 동기화의 기준)
    input wire rst_n,               // 시스템 초기화 비동기 리셋 핀 (Active-Low)

    input wire [10:0] img_width,    // 현재 레이어의 실제 이미지 가로 폭 (랩어라운드 기준값)

    input wire [DATA_WIDTH-1:0] pixel_in, // 카메라/DMA에서 들어오는 최신 1픽셀 데이터
    input wire valid_in,                  // pixel_in 데이터가 유효함을 알리는 AXI TVALID 신호

    input wire ready_in,                  // 뒷단(윈도우 생성기)에서 보내는 멈춤(Stall) 신호
    output wire ready_out,                // 앞단으로 나의 바쁨 상태를 알리는 AXI TREADY 신호

    // 1클럭 지연되어 완벽하게 위상이 맞춰진 6개의 줄(Row) 출력
    output reg [DATA_WIDTH-1:0] row0_out, // 가장 오래된 5줄 전 데이터
    output reg [DATA_WIDTH-1:0] row1_out, // 4줄 전 데이터
    output reg [DATA_WIDTH-1:0] row2_out, // 3줄 전 데이터
    output reg [DATA_WIDTH-1:0] row3_out, // 2줄 전 데이터
    output reg [DATA_WIDTH-1:0] row4_out, // 1줄 전 데이터
    output reg [DATA_WIDTH-1:0] row5_out, // 방금 들어온 최신 데이터 (1클럭 지연)
    
    output reg valid_out                  // 출력되는 6줄의 데이터가 유효함을 알리는 깃발
);

    // -------------------------------------------------------------------------
    // [1] 배압 신호 연결 (Backpressure)
    // 설명: 라인 버퍼 자체는 멈춤 판단을 하지 않고, 뒷단의 상태를 앞단으로 그대로 패스합니다.
    // -------------------------------------------------------------------------
    assign ready_out = ready_in;

    // -------------------------------------------------------------------------
    // [2] 블록 램(BRAM) 모사 배열 선언
    // 설명: Vivado 합성기가 이 배열들을 100% 하드웨어 BRAM으로 추론하도록 유도합니다.
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram0 [0:MAX_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram1 [0:MAX_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram2 [0:MAX_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram3 [0:MAX_WIDTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram4 [0:MAX_WIDTH-1];

    // -------------------------------------------------------------------------
    // [3] 주소 및 제어용 지연 레지스터 선언
    // -------------------------------------------------------------------------
    reg [10:0] col_ptr;      // 현재 픽셀이 저장될 BRAM의 X좌표 주소 (쓰기 및 읽기 포인터)
    reg [10:0] col_ptr_d1;   // Cascade 이동을 위해 1클럭 지연시킨 포인터
    reg valid_in_d1;         // Cascade 이동을 위해 1클럭 지연시킨 Valid 신호

    // -------------------------------------------------------------------------
    // [4] 파이프라인 동기화 블록 (Always Block)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        // 비동기 리셋: 시스템이 0일 때 모든 제어용 레지스터와 출력을 초기화합니다.
        // BRAM 내부는 하드웨어 특성상 초기화할 필요가 없습니다.
        if (!rst_n) begin
            valid_out   <= 0;
            valid_in_d1 <= 0;
            col_ptr     <= 0;
            col_ptr_d1  <= 0;
            row0_out    <= 0; row1_out <= 0; row2_out <= 0;
            row3_out    <= 0; row4_out <= 0; row5_out <= 0;
        end 
        // 하드웨어가 동작 상태일 때 (ready_in == 1, 즉 Stall이 아닐 때)
        else if (ready_in) begin
            
            // 💡 [핵심 동기화 1] Cascade를 위한 1클럭 지연 백업
            valid_in_d1 <= valid_in;
            col_ptr_d1  <= col_ptr;
            
            // 데이터들이 읽히는 데 1클럭이 걸리므로, valid_out 역시 1클럭 지연시킵니다.
            valid_out   <= valid_in; 

            // =================================================================
            // Phase 1: BRAM에서 읽고 출력 핀에 래치 (1-Clock Latency)
            // =================================================================
            if (valid_in) begin
                // BRAM에서 현재 주소의 값을 꺼내 출력 레지스터에 바로 꽂습니다.
                row4_out <= ram4[col_ptr];
                row3_out <= ram3[col_ptr];
                row2_out <= ram2[col_ptr];
                row1_out <= ram1[col_ptr];
                row0_out <= ram0[col_ptr];
                
                // 최신 픽셀도 BRAM 읽기 속도와 똑같이 1클럭 지연시켜 위상을 완벽히 맞춥니다.
                row5_out <= pixel_in;

                // 방금 들어온 최신 픽셀은 가장 윗단 버퍼(ram4)에 새롭게 저장합니다.
                ram4[col_ptr] <= pixel_in;
                
                // 가로 한 줄(img_width)을 다 채우면 주소를 다시 0으로 돌립니다.
                col_ptr <= (col_ptr == img_width - 1) ? 0 : col_ptr + 1;
            end

            // =================================================================
            // Phase 2: 데이터 폭포수 (Cascading) 하강
            // =================================================================
            // 1클럭 전에 읽어낸 과거 줄 데이터들을 다음 BRAM으로 밀어 내립니다.
            // 이때 주소 역시 1클럭 전의 주소(col_ptr_d1)를 사용해야 엇갈리지 않습니다.
            if (valid_in_d1) begin
                ram3[col_ptr_d1] <= row4_out;
                ram2[col_ptr_d1] <= row3_out;
                ram1[col_ptr_d1] <= row2_out;
                ram0[col_ptr_d1] <= row1_out;
            end
        end
    end
endmodule