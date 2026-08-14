`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: channel_folder_3x3
// Description: 
//   - Window Generator에서 넘어온 거대한 3x3 윈도우(MAX_CH 기준, 최대 18,432-bit)를 받아
//     현재 레이어의 'total_ch_in' 스펙에 맞춰 하위 채널(FOLD_CH, 예: 16채널) 크기로 
//     여러 번 잘라서(Folding) 순차 전송하는 시분할(Time-Multiplexing) 라우팅 모듈.
//
//   [아키텍처 핵심 기능 및 수술 내역]
//   1. 파이프라인 동기화 (1-Clock Alignment): 이전 버전에서 출력 인덱스(fold_idx)가 
//      데이터보다 1클럭 먼저 튀어나가던 치명적 엇박자 버그를 수술했습니다. 
//      내부 상태 추적용 카운터(internal_fold_cnt)와 출력 포트(fold_idx)를 물리적으로 분리하여,
//      데이터, Valid, Last Fold, Index 4개의 신호가 완벽하게 한날한시에 쏟아지도록 동기화했습니다.
//   2. AXI-Stream 백프레셔 프리즈 (Freeze): 뒷단 연산기(Conv Array)가 멈춤 신호(ready_in=0)를
//      보내면, 현재 썰어낸 조각을 덮어쓰거나 날려버리지 않고 그 상태 그대로 파이프라인을 얼립니다.
//   3. 동적 채널 대응 (Dynamic Reconfiguration): 레이어마다 바뀌는 total_ch_in을 받아 
//      스스로 폴딩 횟수(current_max_folds)를 계산하여 불필요한 공회전을 막습니다.
//////////////////////////////////////////////////////////////////////////////////

module channel_folder_3x3 #(
    parameter DATA_WIDTH = 8,   // 8비트 양자화 픽셀
    parameter MAX_CH = 256,     // 지원 가능한 최대 입력 채널 수
    parameter FOLD_CH = 16,     // 한 번에 연산할 채널 수 (병렬도, Conv 코어 개수와 일치)
    parameter FIDX_W = 4        // = clog2(MAX_CH/FOLD_CH), V2001
)(
    input wire clk,
    input wire rst_n,
    
    // [Dynamic Configuration] Top 래퍼에서 설정하는 현재 레이어의 활성 채널 수
    input wire [9:0] total_ch_in, 
    
    // [Input] Window Generator로부터 들어오는 18,432-bit 거대 윈도우
    input wire [DATA_WIDTH*MAX_CH*9-1:0] window_in,
    input wire valid_in,
    
    // [Control] 백프레셔 핸드셰이크 신호
    input wire ready_in,        // 뒷단(Conv Array)에서 오는 멈춤 신호 (0이면 전송 멈춤)
    output reg ready_out,       // 앞단(Window Gen)으로 보내는 멈춤 신호 (0이면 데이터 수신 거부)
    
    // [Output] Conv 연산기로 들어갈 16채널(1152-bit) 윈도우 조각
    output reg [DATA_WIDTH*FOLD_CH*9-1:0] folded_window_out,
    output reg valid_out,
    output reg last_fold_out,   // 1: 현재 윈도우의 마지막 조각임 (누적기 초기화 및 출력 지시용)
    output reg [FIDX_W:0] fold_idx  // 현재 출력되는 조각의 인덱스 번호
);

    // =========================================================================
    // [1] 내부 파라미터 및 버퍼 레지스터 선언
    // =========================================================================
    localparam MAX_PIXEL_BITS = DATA_WIDTH * MAX_CH; 
    localparam FOLD_BITS = DATA_WIDTH * FOLD_CH;

    // 18,432-bit 데이터를 픽셀(9개) 단위로 쪼개어 담아둘 거대한 레지스터 버퍼
    reg [MAX_PIXEL_BITS-1:0] window_buffer [0:8];
    
    // 상태 레지스터 (0: IDLE, 1: FOLDING)
    reg busy;

    // 현재 레이어 스펙에 맞춘 총 폴딩 횟수 (예: 64채널이면 64/16 = 4번)
    reg [FIDX_W:0] current_max_folds;
    
    // 💡 [핵심 수술] 내부 상태 추적용 카운터
    // 출력 포트(fold_idx)와 분리되어 콤비네이셔널 로직의 참조용으로만 사용됩니다.
    reg [FIDX_W:0] internal_fold_cnt;

    integer i;
    
    // =========================================================================
    // [2] 콤비네이셔널 데이터 슬라이서 (Combinational Slicer)
    // - 내부 카운터(internal_fold_cnt)를 기반으로 거대 버퍼에서 16채널씩 잘라내어 이어 붙입니다.
    // =========================================================================
    reg [DATA_WIDTH*FOLD_CH*9-1:0] temp_folded_out;

    always @(*) begin
        for(i=0; i<9; i=i+1) begin
            // 9개의 픽셀 각각에 대해, 현재 폴딩 순서에 해당하는 채널 위치만 핀셋으로 잘라냄
            temp_folded_out[i*FOLD_BITS +: FOLD_BITS] = window_buffer[i][internal_fold_cnt * FOLD_BITS +: FOLD_BITS];
        end
    end

    // =========================================================================
    // [3] 순차 회로 (FSM 및 파이프라인 동기화 로직)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            ready_out <= 1; // 리셋 시 언제든 받을 준비가 되어있음
            valid_out <= 0;
            last_fold_out <= 0;
            fold_idx <= 0;
            internal_fold_cnt <= 0; 
            folded_window_out <= 0;
            current_max_folds <= 0;
            for(i = 0; i < 9; i = i + 1) begin
                window_buffer[i] <= 0;
            end
        end 
        else begin
            // -----------------------------------------------------------------
            // [State 1] IDLE: 앞단에서 새로운 거대 윈도우가 들어오길 기다림
            // -----------------------------------------------------------------
            if (!busy) begin
                if (valid_in) begin
                    // 1. 최대 폴딩 횟수 동적 계산 및 갱신
                    current_max_folds <= total_ch_in / FOLD_CH;
                    
                    // 2. 입력된 거대 윈도우를 안전하게 9개의 내부 버퍼로 캡처
                    for(i=0; i<9; i=i+1) begin
                        window_buffer[i] <= window_in[i*MAX_PIXEL_BITS +: MAX_PIXEL_BITS];
                    end
                    
                    // 3. 상태를 바쁘게(busy) 바꾸고 앞단(Window Gen)을 멈춤(ready_out=0)
                    busy <= 1;
                    ready_out <= 0; 
                    
                    // 4. 내부 및 외부 포인터 초기화
                    internal_fold_cnt <= 0; 
                    valid_out <= 0;
                    last_fold_out <= 0;
                    fold_idx <= 0; 
                end 
                else begin
                    // 데이터가 안 들어오면 계속 문을 열고 대기
                    ready_out <= 1;
                    valid_out <= 0;
                    last_fold_out <= 0;
                end
            end 
            // -----------------------------------------------------------------
            // [State 2] FOLDING: 잘라낸 데이터를 뒷단(Conv)으로 순차 전송
            // -----------------------------------------------------------------
            else begin
                // 💡 [백프레셔 프리즈] 뒷단이 데이터를 받을 준비가 안 되었다면(ready_in==0)
                // if문 안을 비워둠으로써 모든 레지스터(출력 데이터, 인덱스, 카운터)가 
                // 다음 클럭이 뛸 때까지 완벽하게 멈춘 상태를 유지합니다. (데이터 유실 방어)
                if (valid_out && !ready_in) begin
                    // Freeze State
                end
                else begin
                    if (internal_fold_cnt < current_max_folds) begin
                        // 💡 [파이프라인 동기화 수술]
                        // 데이터, 인덱스, Valid, Last Fold 신호가 플립플롭을 통해
                        // 정확히 1클럭 지연되어 동일한 타이밍에 밖으로 쏟아져 나갑니다.
                        folded_window_out <= temp_folded_out;
                        fold_idx          <= internal_fold_cnt; // 출력 포트가 썰어낸 데이터와 완벽 일치!
                        valid_out         <= 1;
                        
                        // 현재 조각이 마지막 조각이라면 Last Fold 플래그를 세움
                        if (internal_fold_cnt == current_max_folds - 1) begin
                            last_fold_out <= 1;
                        end else begin
                            last_fold_out <= 0;
                        end
                        
                        // 전송을 준비했으니 내부 카운터는 다음 조각을 썰기 위해 1칸 전진
                        internal_fold_cnt <= internal_fold_cnt + 1; 
                    end 
                    else begin
                        // 모든 폴딩 전송이 완료됨 -> 상태를 IDLE로 되돌리고 문을 다시 염
                        busy <= 0;
                        valid_out <= 0;
                        last_fold_out <= 0;
                        fold_idx <= 0;
                        internal_fold_cnt <= 0;
                        ready_out <= 1; // "다 잘라서 보냈다! 앞 모듈아 다음 픽셀 윈도우 보내라!"
                    end
                end
            end
        end
    end
endmodule