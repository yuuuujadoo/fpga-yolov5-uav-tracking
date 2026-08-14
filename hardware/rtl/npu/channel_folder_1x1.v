`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: channel_folder_1x1
// Description: 
//   - 1x1 Conv 전용 채널 폴더 모듈입니다.
//   - 윈도우(9픽셀) 단위가 아닌 단일 픽셀의 전체 채널 데이터(최대 256ch)를 한 번에 받아,
//     병렬 연산기 크기(FOLD_CH = 16채널)에 맞춰 무썰듯 잘라내어 순차 전송합니다.
//   - [공간 다이어트] 3x3의 9픽셀 2D 배열을 삭제하고, 단일 1D 거대 레지스터로 최적화했습니다.
//   - [핵심 수술 내역: Absolute Freeze] 다운스트림(Conv 배열)이 멈추면(ready_in=0), 
//     현재 출력의 유효성(valid_out)과 무관하게 무조건 내부 상태를 동결(Freeze)시켜
//     데이터가 엇갈리는 Stall Slip 버그를 원천 차단했습니다.
//////////////////////////////////////////////////////////////////////////////////

module channel_folder_1x1 #(
    parameter DATA_WIDTH = 8,   // 양자화된 픽셀 데이터 비트 수 (INT8)
    parameter MAX_CH     = 256, // 시스템이 지원하는 최대 입력 채널 수
    parameter FOLD_CH    = 16,  // 한 번에 잘라내어 연산할 채널 수 (하드웨어 병렬도)
    parameter FIDX_W     = 4    // = clog2(MAX_CH/FOLD_CH), V2001
)(
    input wire clk,
    input wire rst_n,
    
    // [Dynamic Configuration] 
    // Top 모듈(소프트웨어)에서 현재 처리 중인 레이어의 실제 입력 채널 수를 알려줍니다.
    input wire [9:0] total_ch_in, 
    
    // [Data Flow: Input] 
    // 앞단에서 들어오는 거대한 1픽셀 데이터 (예: 256ch * 8bit = 2048-bit)
    input wire [(DATA_WIDTH * MAX_CH)-1:0] pixel_in,
    input wire valid_in,
    
    // [Control: Handshake] 
    input wire ready_in,        // 뒷단(MAC 배열)에서 보내는 정지 신호 (0: 얼음, 1: 땡)
    output reg ready_out,       // 앞단(픽셀 공급처)으로 보내는 정지 신호
    
    // [Data Flow: Output] 
    // 병렬 연산기(16 코어)에 맞게 잘라낸 128-bit (16ch * 8bit) 데이터 조각
    output reg [(DATA_WIDTH * FOLD_CH)-1:0] folded_pixel_out,
    output reg valid_out,
    
    // [Control: Output]
    output reg last_fold_out,   // 1이면 현재 픽셀의 마지막 채널 조각임을 뒷단에 알림 (편향 덧셈용)
    output reg [FIDX_W:0] fold_idx // 현재 잘라내고 있는 조각의 순번 (디버깅 및 외부 SRAM 지연 매핑용)
);

    // 내부 파라미터 (버스 크기 계산)
    localparam MAX_PIXEL_BITS = DATA_WIDTH * MAX_CH;
    localparam FOLD_BITS      = DATA_WIDTH * FOLD_CH;

    // 단일 픽셀의 전체 데이터를 담아둘 거대 버퍼 레지스터
    reg [MAX_PIXEL_BITS-1:0] pixel_buffer;
    reg busy; // FSM 상태 플래그 (0: IDLE, 1: FOLDING)
    
    // 런타임에 동적으로 계산되는 총 폴딩 횟수 레지스터
    reg [FIDX_W:0] current_max_folds;

    // =========================================================================
    // [조합 회로] 동적 데이터 슬라이싱 (Combinational Slicing)
    // =========================================================================
    // for 루프 없이 Verilog의 부분 선택(Part-select) 문법을 활용하여
    // 2048-bit 버스에서 현재 fold_idx 위치의 128-bit만 즉시 뚝 잘라냅니다.
    // 이는 합성 시 거대한 Multiplexer(MUX)로 구현됩니다.
    wire [FOLD_BITS-1:0] temp_folded_out;
    assign temp_folded_out = pixel_buffer[fold_idx * FOLD_BITS +: FOLD_BITS];

    // =========================================================================
    // [순차 회로] FSM 및 AXI-Stream 제어 로직
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 0;
            ready_out <= 1; // 초기 상태는 데이터 수신 가능
            valid_out <= 0;
            last_fold_out <= 0;
            fold_idx <= 0;
            folded_pixel_out <= 0;
            current_max_folds <= 0;
            pixel_buffer <= 0;
        end 
        else begin
            // -----------------------------------------------------------------
            // [State 1] IDLE: 새로운 픽셀 데이터를 기다림
            // -----------------------------------------------------------------
            if (!busy) begin
                if (valid_in) begin
                    // 1. 현재 레이어 채널 수에 맞춰 총 폴딩 횟수를 계산하여 캡처
                    current_max_folds <= total_ch_in / FOLD_CH;
                    
                    // 2. 2048-bit 거대 픽셀 데이터를 내부 버퍼에 통째로 저장
                    pixel_buffer <= pixel_in;
                    
                    // 3. 상태 전환 및 Handshake 방어
                    busy <= 1;
                    ready_out <= 0; // 자르기(Folding)를 시작하므로 앞단 데이터 유입 완전 차단
                    fold_idx <= 0;
                    valid_out <= 0;
                    last_fold_out <= 0;
                end 
                else begin
                    // 데이터가 안 들어오면 계속 대기
                    ready_out <= 1;
                    valid_out <= 0;
                    last_fold_out <= 0;
                end
            end 
            
            // -----------------------------------------------------------------
            // [State 2] FOLDING: 버퍼에 담긴 데이터를 순차적으로 잘라서 전송
            // -----------------------------------------------------------------
            else begin
                // 💡 [핵심 수술: Absolute Freeze] 
                // 다운스트림(Conv Array)이 바쁘다며 ready_in = 0을 보내면,
                // 현재 데이터 출력 여부(valid_out)와 상관없이 모든 내부 카운터와
                // 레지스터의 상태를 갱신하지 않고(Freeze) 다음 클럭까지 버팁니다.
                if (!ready_in) begin
                    // 아무 동작도 하지 않음으로써 이전 값을 100% 홀드(Hold)
                end
                
                // 뒷단이 받을 준비가 되어 있을 때만 폴딩 전진
                else begin
                    // 아직 남은 조각이 있다면
                    if (fold_idx < current_max_folds) begin
                        // 조합 회로에서 잘라둔 128-bit 데이터를 출력 레지스터에 등록
                        folded_pixel_out <= temp_folded_out;
                        valid_out <= 1;
                        
                        // 현재 조각이 마지막 조각인지 판단하여 플래그 셋팅
                        if (fold_idx == current_max_folds - 1) begin
                            last_fold_out <= 1;
                        end else begin
                            last_fold_out <= 0;
                        end
                        
                        // 다음 조각을 위해 인덱스 1칸 전진
                        fold_idx <= fold_idx + 1; 
                    end 
                    // 모든 조각을 다 보냈다면
                    else begin
                        busy <= 0;             // 다시 IDLE 상태로 복귀
                        valid_out <= 0;
                        last_fold_out <= 0;
                        fold_idx <= 0;
                        ready_out <= 1;        // 앞단에 다음 픽셀을 달라고 허가(ready_out=1)
                    end
                end
            end
        end
    end
endmodule