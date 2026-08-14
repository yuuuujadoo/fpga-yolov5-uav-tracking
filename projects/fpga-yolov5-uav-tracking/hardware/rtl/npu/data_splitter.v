`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 모듈명: data_splitter (48-bit to 6x8-bit Router)
// 역할: 
//   Formatter에서 넘어온 48-bit 뭉치 데이터를 Bounding Box Decoder가 편하게 쓸 수 있도록
//   tx, ty, tw, th, conf, cls 각각 8-bit씩 6가닥으로 찢어주는(Slicing) 역할을 합니다.
//   (복잡한 논리 연산 없이, 전선을 분배해주는 배전반 같은 모듈입니다)
//////////////////////////////////////////////////////////////////////////////////

module data_splitter #(
    // [정적 파라미터 선언부]
    parameter DATA_WIDTH = 8,       // 1개 변수의 비트 폭 (8-bit 양자화)
    parameter CHANNELS = 6,         // 입력 데이터에 포함된 요소의 개수 (6개 파라미터)
    parameter MAX_COORD_BITS = 8    // P3 해상도(80)를 표현하기 위한 좌표 레지스터 폭 (8-bit)
)(
    // [시스템 공통 포트]
    input  wire                               clk,            // 메인 시스템 클럭
    input  wire                               rst_n,          // Active-Low 리셋 신호

    // [Formatter로부터 들어오는 입력 포트]
    input  wire                               i_valid,        // 입력 데이터(i_data)가 쓸모 있는 값인지 알려주는 신호
    input  wire [(DATA_WIDTH*CHANNELS)-1:0]   i_data,         // 6채널이 한 덩어리로 뭉쳐진 48-bit 패키지 데이터
    input  wire [MAX_COORD_BITS-1:0]          i_cx,           // 해당 패키지의 X 그리드 좌표
    input  wire [MAX_COORD_BITS-1:0]          i_cy,           // 해당 패키지의 Y 그리드 좌표
    input  wire [1:0]                         i_anchor,       // 해당 패키지의 앵커 식별자

    // [다운스트림 제어 포트]
    input  wire                               ready_in,       // 💡[버그 수정] 다음 단이 막혔을 때 0이 되어 상태를 동결시키는 신호

    // [Decoder 방향으로 찢어져서 나가는 출력 포트]
    output reg                                o_valid,        // 찢어낸 데이터들을 연산해도 좋다고 디코더에 알리는 신호
    output reg  [DATA_WIDTH-1:0]              o_tx,           // 중심 X 변위를 계산할 8-bit 원시 데이터
    output reg  [DATA_WIDTH-1:0]              o_ty,           // 중심 Y 변위를 계산할 8-bit 원시 데이터
    output reg  [DATA_WIDTH-1:0]              o_tw,           // 박스 너비 비율을 계산할 8-bit 원시 데이터
    output reg  [DATA_WIDTH-1:0]              o_th,           // 박스 높이 비율을 계산할 8-bit 원시 데이터
    output reg  [DATA_WIDTH-1:0]              o_tconf,        // 해당 박스에 사물이 있을 확률 (Bypass)
    output reg  [DATA_WIDTH-1:0]              o_tcls,         // 해당 사물이 무슨 클래스인지 맞출 확률 (Bypass)
    
    // 계산기(DSP)에 들어가지 않고 그대로 지나가는 Bypass 좌표 메타데이터
    output reg  [MAX_COORD_BITS-1:0]          o_cx,           // 1클럭 딜레이 된 X 좌표 복사본
    output reg  [MAX_COORD_BITS-1:0]          o_cy,           // 1클럭 딜레이 된 Y 좌표 복사본
    output reg  [1:0]                         o_anchor        // 1클럭 딜레이 된 앵커 식별자 복사본
);

    // =========================================================================
    // [always 블록] 데이터 Slicing 및 Latch (순차 회로)
    // 설명: 클럭의 상승 에지에 맞춰 48-bit를 8-bit 전용 레지스터 6개로 찢어서 할당합니다.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 리셋 시 파이프라인의 모든 쓰레기 데이터를 0으로 씻어냅니다.
            o_valid <= 1'b0;
            o_tx <= 0; o_ty <= 0; o_tw <= 0; o_th <= 0; o_tconf <= 0; o_tcls <= 0;
            o_cx <= 0; o_cy <= 0; o_anchor <= 0;
        end else begin
            // 💡[버그 수정] 다운스트림 모듈들이 데이터 받을 준비가 되었을 때(ready_in == 1)만 동작합니다.
            // ready_in이 0이라면 이 블록 안으로 들어오지 않으므로, 레지스터들이 이전 값을 100% 유지(Freeze)합니다.
            if (ready_in) begin
                
                // 만약 앞단에서 유효한 데이터를 주었다면 (i_valid == 1)
                if (i_valid) begin
                    // 나도 유효한 데이터를 찢어냈으므로 1클럭 지연된 유효 신호(1)를 출력합니다.
                    o_valid <= 1'b1; 

                    // [데이터 슬라이싱 로직] 파라미터 수식 기반으로 유연하게 비트를 잘라냅니다.
                    // 1x1 Conv에서 채널이 쌓인 순서대로 하위 비트부터 하나씩 전용 선에 할당합니다.
                    
                    // 0~7번 비트 추출 -> tx 할당
                    o_tx    <= i_data[DATA_WIDTH*1 - 1 : DATA_WIDTH*0];  
                    // 8~15번 비트 추출 -> ty 할당
                    o_ty    <= i_data[DATA_WIDTH*2 - 1 : DATA_WIDTH*1];  
                    // 16~23번 비트 추출 -> tw 할당
                    o_tw    <= i_data[DATA_WIDTH*3 - 1 : DATA_WIDTH*2];  
                    // 24~31번 비트 추출 -> th 할당
                    o_th    <= i_data[DATA_WIDTH*4 - 1 : DATA_WIDTH*3];  
                    // 32~39번 비트 추출 -> conf 할당
                    o_tconf <= i_data[DATA_WIDTH*5 - 1 : DATA_WIDTH*4];  
                    // 40~47번 비트 추출 -> cls 할당
                    o_tcls  <= i_data[DATA_WIDTH*6 - 1 : DATA_WIDTH*5];  

                    // 좌표와 앵커 식별자는 연산이 필요 없으므로 그냥 바이패스 레지스터에 복사합니다.
                    o_cx     <= i_cx;
                    o_cy     <= i_cy;
                    o_anchor <= i_anchor;
                    
                end else begin
                    // 앞단 데이터가 무효하다면, 뒤따라오는 수학 계산기들이 
                    // 쓰레기값으로 헛연산을 하지 않도록 유효 신호를 꺼버립니다.
                    o_valid <= 1'b0;
                end
                
            end
        end
    end

endmodule