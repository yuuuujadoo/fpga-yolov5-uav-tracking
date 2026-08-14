`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: maxpool_5x5_array
// Description: 
//   - 단일 채널 max_pooling_5x5 코어를 FOLD_CH(16)개 병렬로 인스턴스화하는 배열 래퍼.
//   - [데이터 분배 조향] 3,200-bit 덩어리 데이터를 채널별 200-bit씩 우아하게 쪼개서 분배.
//   - [동시 정지] ready_in 신호를 브로드캐스팅하여 전체 코어 동시 프리즈(Freeze) 지원.
//////////////////////////////////////////////////////////////////////////////////

module maxpool_5x5_array #(
    parameter DATA_WIDTH = 8,
    parameter FOLD_CH    = 16   // 한 클럭에 동시 처리할 채널 수 (병렬도)
)(
    input wire clk,
    input wire rst_n,

    // [Input] Channel Folder로부터 들어오는 16채널 분량의 5x5 윈도우 데이터
    // 크기: 25픽셀 * 16채널 * 8비트 = 3,200 bits
    // 패킹 구조: [픽셀0_16채널] -> [픽셀1_16채널] -> ... -> [픽셀24_16채널]
    input wire [DATA_WIDTH*FOLD_CH*25-1:0] window_in,
    
    // [Control Signals]
    input wire valid_in,
    input wire last_fold_in,
    input wire ready_in,
    
    // [Outputs] 각 채널별 최댓값(1픽셀 * 16채널 * 8비트 = 128 bits)
    output wire [DATA_WIDTH*FOLD_CH-1:0] pixel_out,
    output wire valid_out,
    output wire last_fold_out
);

    wire [DATA_WIDTH-1:0] w_max_out      [0:FOLD_CH-1];
    wire                  w_valid_out    [0:FOLD_CH-1];
    wire                  w_last_fold_out[0:FOLD_CH-1];

    // =========================================================================
    // 개별 채널용 코어(Core) 16개 인스턴스화
    // =========================================================================
    genvar c, p;
    generate
        for (c = 0; c < FOLD_CH; c = c + 1) begin : MAX_POOL_CORES
            
            // 💡 [핵심 데이터 분배기]
            // 25개의 픽셀 묶음 안에서, '현재 채널(c)'에 해당하는 데이터만 
            // 25번 뽑아내어 200-bit 짜리 전용 윈도우를 조립합니다.
            wire [DATA_WIDTH*25-1:0] ch_specific_window;
            
            for (p = 0; p < 25; p = p + 1) begin : GATHER_PIXELS
                // 인덱스: (픽셀 인덱스 * 채널 병렬도 + 현재 채널) * 데이터 폭
                assign ch_specific_window[p*DATA_WIDTH +: DATA_WIDTH] = 
                       window_in[(p*FOLD_CH + c)*DATA_WIDTH +: DATA_WIDTH];
            end
            
            // 조립된 전용 윈도우를 개별 코어에 연결
            max_pooling_5x5 #(
                .DATA_WIDTH(DATA_WIDTH)
            ) u_max_core (
                .clk           (clk),
                .rst_n         (rst_n),
                .window_in     (ch_specific_window), 
                .valid_in      (valid_in),
                .last_fold_in  (last_fold_in),
                .ready_in      (ready_in),
                .max_out       (w_max_out[c]),
                .valid_out     (w_valid_out[c]),
                .last_fold_out (w_last_fold_out[c])
            );
        end
    endgenerate

    // =========================================================================
    // 출력 데이터 패킹 (Packing) 및 대표 제어 신호 추출
    // =========================================================================
    generate
        for (c = 0; c < FOLD_CH; c = c + 1) begin : PACK_OUT
            assign pixel_out[c*DATA_WIDTH +: DATA_WIDTH] = w_max_out[c];
        end
    endgenerate

    // 16개의 코어가 완벽히 똑같은 파이프라인 지연을 가지므로, 0번 코어의 신호를 대표로 사용
    assign valid_out     = w_valid_out[0];
    assign last_fold_out = w_last_fold_out[0];

endmodule