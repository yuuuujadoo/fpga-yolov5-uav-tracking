`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: conv_block_3x3 (Pure MAC Core + Auto-Flush + 💡 6x6 Reuse)
//
// Description: 
//   - Line Buffer, Window Generator, Channel Folder, 3x3 Conv Array를 
//     하나로 묶어놓은 범용 3x3 합성곱 연산 통합 래퍼(Wrapper)입니다.
//   - [Auto-Prime & Flush FSM 내장] 외부 제어기의 개입 없이 스스로 
//     프레임 시작 전 빈 줄을 패딩하고, 프레임 종료 후 남은 픽셀을 밀어냅니다.
//
//   [💡 극한의 하드웨어 재사용 (Layer 0 Bypass 전략)]
//   - Stem 레이어(6x6, 3채널)는 108번의 MAC 연산이 필요하며, 
//     본 3x3 Conv 코어(16채널)는 144번의 MAC 연산 수용량을 가집니다. 
//   - is_layer_0=1 신호가 들어오면, 전력을 아끼기 위해 내부 라인 버퍼와 폴더를 끄고,
//     외부에서 들어온 1152-bit(Zero-padded) 데이터를 MAC Array의 입구로 직행(Bypass)시킵니다.
//     이를 통해 단 1개의 코어 배열로 6x6 연산까지 1클럭 만에 처리해 내는 마법을 부립니다.
//////////////////////////////////////////////////////////////////////////////////

module conv_block_3x3 #(
    // -------------------------------------------------------------------------
    // [파라미터 선언부] 하드웨어 아키텍처 스펙 정의
    // -------------------------------------------------------------------------
    parameter DATA_WIDTH = 8,   // 픽셀 및 가중치의 데이터 비트 폭 (INT8)
    parameter BIAS_WIDTH = 16,  // 편향의 데이터 비트 폭 (INT16)
    parameter PSUM_WIDTH = 32,  // 오버플로우 방지용 누적기 비트 폭 (INT32)
    parameter MAX_CH = 256,     // 시스템이 수용할 수 있는 최대 입력 채널 수
    parameter MAX_WIDTH = 320,  // 라인버퍼 내부 BRAM의 최대 가로 지원 길이
    parameter T0 = 16,          // 1클럭당 출력되는 병렬 채널 수 (MAC 코어 개수)
    parameter FOLD_CH = 16,     // 1클럭당 처리(폴딩)되는 병렬 채널 수
    parameter WLAT = 1          // weight 경로 지연 보상 (folded_window 를 이만큼 지연)
)(
    // -------------------------------------------------------------------------
    // [시스템 공통 포트] 클럭 및 리셋
    // -------------------------------------------------------------------------
    input wire clk,             // 시스템 메인 클럭
    input wire rst_n,           // 비동기 리셋 신호 (0일 때 시스템 초기화)
    
    // -------------------------------------------------------------------------
    // [동적 설정 포트] 레이어마다 달라지는 설정값들 (FSM이나 CPU가 제어)
    // -------------------------------------------------------------------------
    input wire [10:0] img_width,  // 현재 처리 중인 이미지의 가로 크기
    input wire [10:0] img_height, // 현재 처리 중인 이미지의 세로 크기
    input wire [9:0]  total_ch_in,// 현재 레이어의 전체 입력 채널 수 (폴딩 횟수 결정용)
    input wire [1:0]  stride,     // 보폭 설정 (1: 촘촘히, 2: 한 칸씩 건너뜀)
    
    // -------------------------------------------------------------------------
    // [Layer 0 전용 바이패스 포트] 6x6 Stem 연산을 위한 직결 통로
    // -------------------------------------------------------------------------
    input wire        is_layer_0,    // 1이면 내부 폴더를 끄고 외부 데이터를 직결(Bypass)
    input wire [1151:0] ext_window_in, // 외부에서 조립되어 들어오는 6x6(3채널) 제로 패딩 데이터
    input wire        ext_valid_in,  // 외부 바이패스 데이터가 유효함을 알리는 신호
    
    // -------------------------------------------------------------------------
    // [일반 3x3 입력 스트림 포트] Line Buffer로 들어가는 일반 경로
    // -------------------------------------------------------------------------
    input wire [DATA_WIDTH*MAX_CH-1:0] pixel_in, // 2048-bit 픽셀 데이터 뭉치
    input wire valid_in,                         // 픽셀 데이터 유효 신호
    
    // -------------------------------------------------------------------------
    // [가중치 및 편향 입력 포트] 외부 SRAM/BRAM에서 0-Clock으로 인가되는 파라미터
    // -------------------------------------------------------------------------
    input wire [DATA_WIDTH*FOLD_CH*9*T0-1:0] weight_in, // 18432-bit 거대 가중치 버스
    input wire [BIAS_WIDTH*T0-1:0] bias_in,             // 256-bit 편향 버스
    
    // -------------------------------------------------------------------------
    // [흐름 제어 (Handshake) 포트] 파이프라인 정지/동작 제어
    // -------------------------------------------------------------------------
    input wire ready_in,   // 다음 모듈(다운스트림)이 바쁠 때 이 모듈 전체를 멈추는(Stall) 신호
    output wire ready_out, // 이전 모듈(업스트림)로 "내가 꽉 찼으니 그만 보내"라고 알리는 배압 신호

    // -------------------------------------------------------------------------
    // [최종 연산 출력 포트] MAC Array에서 계산이 완료된 정답
    // -------------------------------------------------------------------------
    output wire [PSUM_WIDTH*T0-1:0] psum_out, // 16개 채널의 32-bit 누적합 결과
    output wire [4:0] fold_idx_out,           // 외부에 "n번째 폴딩 가중치를 달라"고 요구하는 주소
    output wire valid_out                     // psum_out이 정답임을 알리는 유효 신호
);

    // =========================================================================
    // [1] 배압 신호(Backpressure) 연결 와이어 선언
    // 설명: 파이프라인 역방향으로 흐르는 ready 신호들을 연결할 전선들입니다.
    // =========================================================================
    wire lb_ready_out; // Line Buffer가 내보내는 ready 신호
    wire wg_ready_out; // Window Gen이 내보내는 ready 신호
    wire cf_ready_out; // Channel Folder가 내보내는 ready 신호

    // =========================================================================
    // [전략 C] 입력채널 시분할용 divide-by-2 위상 (sub-fold 번호)
    //   - 16채널 fold 를 8채널씩 2 cycle 에 나눠 처리하기 위해, channel_folder 가
    //     각 fold 를 2 cycle 동안 hold 하도록 cf_sub 로 게이팅.
    //   - cf_sub: channel_folder 출력 fold 의 sub-fold 번호.
    //       0 = fresh(첫 cycle, ch0-7),  1 = held(둘째 cycle, ch8-15)
    //   - cf_valid_out 가 연속 high 인 동안 매 cycle 토글 -> 2 cycle 마다 fold 진행.
    //     advance(cf_sub=1) / hold(cf_sub=0). cf_sub 와 advance 가 동일 신호원이라 위상 일관.
    // =========================================================================
    reg  cf_sub;
    wire cf_rdy = ready_in & cf_sub;     // channel_folder.ready_in : held(=1)에서만 advance
    reg  stem_sub;                       // stem(is_layer_0) 경로 sub-fold 번호

    // 💡 [다중 충돌 방어 로직] 
    // - is_layer_0 == 1: stem 윈도우를 2 cycle hold. 단, 윈도우가 없는 동안(ext_valid_in=0,
    //     라인버퍼 충전 중)에는 ready_out=ready_in 으로 wrapper_6x6 를 정상 진행시켜야 함.
    //     (그렇지 않으면 wrapper 가 입력까지 멈춰 윈도우를 영영 못 만드는 데드락 발생.)
    //     윈도우가 나온 뒤(ext_valid_in=1)에만 stem_sub 로 fresh(0)=hold / held(1)=advance.
    // - is_layer_0 == 0: 라인 버퍼의 준비 상태(lb_ready_out)를 그대로 외부로 전달.
    assign ready_out = is_layer_0 ? (ready_in & (ext_valid_in ? stem_sub : 1'b1)) : lb_ready_out;

    // =========================================================================
    // [2] 하위 모듈 배선 (Line Buffer -> Window Gen -> Channel Folder)
    // 설명: 픽셀 데이터를 받아 3줄로 정렬하고, 3x3 윈도우로 묶은 뒤, 폴딩 채널로 절단합니다.
    // =========================================================================
    
    // Line Buffer 출력용 와이어
    wire [DATA_WIDTH*MAX_CH-1:0] lb_row0, lb_row1, lb_row2;
    wire lb_valid;
    
    // 인스턴스 1: 라인 버퍼 (3줄 정렬기)
    line_buffer_3x3 #( .DATA_WIDTH(DATA_WIDTH), .MAX_CH(MAX_CH), .MAX_WIDTH(MAX_WIDTH) ) u_line_buffer (
        .clk(clk), .rst_n(rst_n), .img_width(img_width), 
        .pixel_in(pixel_in), .valid_in(valid_in),  // 모듈의 입력 포트를 그대로 받음
        .ready_in(wg_ready_out),                   // 뒷단(윈도우 생성기)의 바쁨 신호를 받음
        .row0_out(lb_row0), .row1_out(lb_row1), .row2_out(lb_row2), 
        .valid_out(lb_valid), 
        .ready_out(lb_ready_out)                   // 앞단으로 나의 상태 송신
    );

    // Window Generator 출력용 와이어
    wire [DATA_WIDTH*MAX_CH*9-1:0] wg_window;
    wire wg_valid;
    
    // 인스턴스 2: 윈도우 생성기 (3x3 절대 좌표 기반 조립기)
    window_generator_3x3 #( .DATA_WIDTH(DATA_WIDTH), .MAX_CH(MAX_CH) ) u_window_gen (
        .clk(clk), .rst_n(rst_n), .img_width(img_width), .img_height(img_height), .stride(stride),
        .row0_in(lb_row0), .row1_in(lb_row1), .row2_in(lb_row2), .valid_in(lb_valid), // LB의 출력 수신
        .ready_in(cf_ready_out),                   // 뒷단(채널 폴더)의 바쁨 신호 수신
        .window_out(wg_window), .valid_out(wg_valid), 
        .ready_out(wg_ready_out)                   // 앞단(LB)으로 나의 상태 송신
    );

    // Channel Folder 출력용 와이어
    wire [DATA_WIDTH*FOLD_CH*9-1:0] cf_folded_window;
    wire cf_valid_out, cf_last_fold;
    
    // 인스턴스 3: 채널 폴더 (시분할 절단기)
    channel_folder_3x3 #( .DATA_WIDTH(DATA_WIDTH), .MAX_CH(MAX_CH), .FOLD_CH(FOLD_CH) ) u_channel_folder (
        .clk(clk), .rst_n(rst_n), .total_ch_in(total_ch_in),
        .window_in(wg_window), .valid_in(wg_valid), // WG의 출력 수신
        .ready_in(cf_rdy),                          // 💡[전략C] divide-by-2: held(cf_sub=1)에서만 advance -> fold 2 cycle hold
        .folded_window_out(cf_folded_window), .valid_out(cf_valid_out), .fold_idx(fold_idx_out),
        .last_fold_out(cf_last_fold), 
        .ready_out(cf_ready_out)                    // 앞단(WG)으로 나의 상태 송신
    );

    // =========================================================================
    // [전략 C] sub-fold 위상 생성
    //   cf_sub : channel_folder 출력이 valid 인 동안 매 cycle 토글(fresh=0/held=1).
    //            cf_valid_out=0(IDLE)일 때 0 으로 리셋 -> 다음 fold 는 항상 fresh(0)부터.
    //   stem_sub: stem(is_layer_0) ext_valid_in 이 high 인 동안 토글.
    // =========================================================================
    always @(posedge clk) begin
        if(!rst_n) begin
            cf_sub <= 1'b0; stem_sub <= 1'b0;
        end else if(ready_in) begin
            // 일반 경로
            if(cf_valid_out) cf_sub <= ~cf_sub; else cf_sub <= 1'b0;
            // stem 경로
            if(ext_valid_in) stem_sub <= ~stem_sub; else stem_sub <= 1'b0;
        end
    end

    // 인스턴스 3 종료. 아래는 weight-window 정렬(WLAT) 및 Layer0 Bypass MUX.
    // =========================================================================
    // [3] 극한의 하드웨어 재사용 (MUX 기반 Layer 0 Bypass 라우팅)
    // 설명: 삼항 연산자(?:)를 사용하여 is_layer_0 플래그에 따라 데이터 흐름을 바꿉니다.
    // =========================================================================
    
    // [weight-window 정렬] 다폴드(다채널) 시 fold_idx 로 요청한 weight 가 ROM 을 거쳐
    //   WLAT 클럭 뒤 도착하므로, folded_window/valid/last_fold/sub 도 WLAT 만큼 지연시켜 정렬.
    //   stem(is_layer_0)은 ext_window 직결(폴드 1개, 지연 불필요).
    reg [1151:0] cf_win_dly [0:2];
    reg          cf_val_dly [0:2];
    reg          cf_lf_dly  [0:2];
    reg          cf_sub_dly [0:2];   // [전략C] sub 위상도 동일 지연
    integer dly_i;
    always @(posedge clk) begin
        if(!rst_n) begin
            for(dly_i=0;dly_i<3;dly_i=dly_i+1) begin cf_win_dly[dly_i]<=0; cf_val_dly[dly_i]<=0; cf_lf_dly[dly_i]<=0; cf_sub_dly[dly_i]<=0; end
        end else if(ready_in) begin
            cf_win_dly[0]<=cf_folded_window; cf_val_dly[0]<=cf_valid_out; cf_lf_dly[0]<=cf_last_fold; cf_sub_dly[0]<=cf_sub;
            cf_win_dly[1]<=cf_win_dly[0];    cf_val_dly[1]<=cf_val_dly[0]; cf_lf_dly[1]<=cf_lf_dly[0]; cf_sub_dly[1]<=cf_sub_dly[0];
            cf_win_dly[2]<=cf_win_dly[1];    cf_val_dly[2]<=cf_val_dly[1]; cf_lf_dly[2]<=cf_lf_dly[1]; cf_sub_dly[2]<=cf_sub_dly[1];
        end
    end
    wire [1151:0] cf_win_aligned = (WLAT<=0)?cf_folded_window : (WLAT==1)?cf_win_dly[0] : (WLAT==2)?cf_win_dly[1] : cf_win_dly[2];
    wire          cf_val_aligned = (WLAT<=0)?cf_valid_out      : (WLAT==1)?cf_val_dly[0] : (WLAT==2)?cf_val_dly[1] : cf_val_dly[2];
    wire          cf_lf_aligned  = (WLAT<=0)?cf_last_fold      : (WLAT==1)?cf_lf_dly[0]  : (WLAT==2)?cf_lf_dly[1]  : cf_lf_dly[2];
    wire          cf_sub_aligned = (WLAT<=0)?cf_sub            : (WLAT==1)?cf_sub_dly[0] : (WLAT==2)?cf_sub_dly[1] : cf_sub_dly[2];

    wire [1151:0] array_window_in = is_layer_0 ? ext_window_in : cf_win_aligned;
    wire          array_valid_in  = is_layer_0 ? ext_valid_in  : cf_val_aligned;
    wire          array_last_fold = is_layer_0 ? 1'b1          : cf_lf_aligned; 
    wire          array_sub       = is_layer_0 ? stem_sub      : cf_sub_aligned; // [전략C] sub-fold 번호

    // =========================================================================
    // [4] 3x3 MAC Array (16채널 병렬 합성곱 코어 엔진)
    // 설명: 16개의 단일 채널 연산기가 복제되어 동시에 곱셈-누적(MAC)을 수행합니다.
    // =========================================================================
    conv_3x3_array #( 
        .T0(T0), .FOLD_CH(FOLD_CH), .DATA_WIDTH(DATA_WIDTH), 
        .BIAS_WIDTH(BIAS_WIDTH), .PSUM_WIDTH(PSUM_WIDTH) 
    ) u_conv_array (
        .clk(clk), .rst_n(rst_n),
        .window_in   (array_window_in),   // MUX를 통과한 최종 윈도우 데이터
        .valid_in    (array_valid_in),    // 데이터 유효 플래그
        .last_fold_in(array_last_fold),   // 누적 종료 플래그
        .sub_in      (array_sub),         // [전략C] sub-fold 번호 (0:ch0-7, 1:ch8-15)
        .weight_in   (weight_in),         // 외부에서 받은 18432-bit 가중치
        .bias_in     (bias_in),           // 외부에서 받은 편향
        .ready_in    (ready_in),          // 외부 멈춤 신호를 16개 코어에 브로드캐스팅
        .psum_out    (psum_out),          // 최종 32-bit * 16개 연산 결과
        .valid_out   (valid_out)          // 연산 완료 알림 플래그
    );

endmodule