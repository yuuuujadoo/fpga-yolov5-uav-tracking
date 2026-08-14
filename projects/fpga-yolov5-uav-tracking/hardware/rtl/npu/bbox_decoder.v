`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// [Bounding Box Decoder 모듈]
// 역할: Sigmoid 배열에서 나온 8-bit 데이터들을 받아, 실제 640x640 이미지 위에서의 
// 16-bit 픽셀 좌표(bx, by)와 박스 크기(bw, bh)로 디코딩(복원)하는 수학 연산 코어입니다.
// [설계 특징]
// 1. P3/P4 재사용: Top 모듈에서 Anchor 크기와 Shift 파라미터만 바꿔주면 무한히 재사용 가능합니다.
// 2. 3-Stage Pipeline: DSP 연산 시간(제곱 -> 앵커 곱셈)에 맞춰 3클럭으로 쪼개져 있으며, 
// 연산하지 않는 conf, cls는 정확히 똑같은 3클럭을 지연시켜 타이밍을 완벽히 동기화합니다.
//////////////////////////////////////////////////////////////////////////////////

module bbox_decoder #(
    // 정적 파라미터 
    parameter DATA_WIDTH = 8,       // 입력 데이터 폭 (INT8)
    parameter COORD_BITS = 8,       // X, Y 그리드 좌표 비트 폭
    parameter OUT_WIDTH  = 16       // 출력 좌표 및 크기 비트 폭 (연산을 거치며 16-bit로 팽창함)
)(
    input  wire                   clk,        // 메인 클럭
    input  wire                   rst_n,      // 비동기 리셋

    // =====================================================================
    // [1. Sigmoid Array로부터 들어오는 데이터 입력 포트]
    // =====================================================================
    input  wire                   i_valid,    // 데이터 유효 신호
    input  wire                   ready_in,   // 💡[버그 수정] 이더넷 FIFO가 가득 찼을 때 0이 되어 연산을 멈추게 하는 신호
    
    input  wire [DATA_WIDTH-1:0]  i_tx,       // Q0.7 스케일이 적용된 X 중심 변위 확률
    input  wire [DATA_WIDTH-1:0]  i_ty,       // Q0.7 스케일이 적용된 Y 중심 변위 확률
    input  wire [DATA_WIDTH-1:0]  i_tw,       // Q0.7 스케일이 적용된 너비 배수
    input  wire [DATA_WIDTH-1:0]  i_th,       // Q0.7 스케일이 적용된 높이 배수
    input  wire [DATA_WIDTH-1:0]  i_conf,     // 객체 신뢰도 (수학 연산 없이 Bypass 됨)
    input  wire [DATA_WIDTH-1:0]  i_cls,      // 클래스 확률 (수학 연산 없이 Bypass 됨)
    
    input  wire [COORD_BITS-1:0]  i_cx,       // 현재 그리드 X 좌표
    input  wire [COORD_BITS-1:0]  i_cy,       // 현재 그리드 Y 좌표
    input  wire [1:0]             i_anchor,   // 현재 박스가 쓸 앵커 번호

    // =====================================================================
    // [2. Top Wrapper에서 쏴주는 동적 제어 파라미터 (런타임 P3/P4 스위칭용)]
    // =====================================================================
    input  wire [3:0]             i_downshift,// 픽셀 스케일 복원용 시프트 횟수 (P3=4, P4=3)
    input  wire [OUT_WIDTH-1:0]   i_pw_0,     // 0번 앵커 너비
    input  wire [OUT_WIDTH-1:0]   i_ph_0,     // 0번 앵커 높이
    input  wire [OUT_WIDTH-1:0]   i_pw_1,     // 1번 앵커 너비
    input  wire [OUT_WIDTH-1:0]   i_ph_1,     // 1번 앵커 높이
    input  wire [OUT_WIDTH-1:0]   i_pw_2,     // 2번 앵커 너비
    input  wire [OUT_WIDTH-1:0]   i_ph_2,     // 2번 앵커 높이

    // =====================================================================
    // [3. 외부 Wrapper단(AXI-Stream 조립부)으로 나가는 16-bit 출력 포트]
    // =====================================================================
    output reg                    o_valid,    // 모든 3-Stage 파이프라인 연산이 끝나고 튀어나온 유효 신호
    output reg signed [OUT_WIDTH-1:0] o_bx,   // 원본 픽셀 단위 X 중심 좌표 (16-bit)
    output reg signed [OUT_WIDTH-1:0] o_by,   // 원본 픽셀 단위 Y 중심 좌표 (16-bit)
    output reg signed [OUT_WIDTH-1:0] o_bw,   // 원본 픽셀 단위 박스 너비 (16-bit)
    output reg signed [OUT_WIDTH-1:0] o_bh,   // 원본 픽셀 단위 박스 높이 (16-bit)
    output reg [DATA_WIDTH-1:0]       o_conf, // 3클럭 동안 방치(Bypass)되어 무사히 시간을 맞춘 신뢰도
    output reg [DATA_WIDTH-1:0]       o_cls   // 3클럭 동안 방치되어 무사히 시간을 맞춘 클래스 확률
);

    // =========================================================================
    // [Pipeline Stage 1] 16-bit 비트 확장 및 기초 상수 세팅
    // =========================================================================
    // 연산 중 오버플로우를 막기 위해 8비트 데이터를 16비트로 넓혀서 담아둘 그릇들
    reg signed [15:0] s1_tx_adj, s1_ty_adj;   // (2*sigmoid - 0.5) 연산 1차 결과 보관용
    reg [15:0] s1_tw_x2, s1_th_x2;            // 박스 크기를 위한 (2*sigmoid) 결과 보관용
    reg [15:0] s1_pw, s1_ph;                  // 현재 앵커 번호에 맞춰 MUX로 선택된 앵커 크기 보관용
    
    // Bypass용 지연(Delay) 레지스터 (Stage 1 방)
    reg [COORD_BITS-1:0] s1_cx, s1_cy;
    reg [DATA_WIDTH-1:0] s1_conf, s1_cls;
    reg s1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 리셋 시 1번 방 완전 소독
            s1_tx_adj <= 0; s1_ty_adj <= 0; s1_tw_x2 <= 0; s1_th_x2 <= 0;
            s1_pw <= 0; s1_ph <= 0; s1_cx <= 0; s1_cy <= 0;
            s1_conf <= 0; s1_cls <= 0; s1_valid <= 0;
        end else if (ready_in) begin // 💡[Freeze 방어] 다음 단이 막히면 1번 방 연산 일시 정지!
            s1_valid <= i_valid;
            
            if (i_valid) begin
                // [X, Y 방정식 풀이 시작]: YOLOv5 공식 = (2 * sigmoid - 0.5)
                // 1. {8'd0, i_tx} 형태로 8비트를 부호 없는 16비트로 강제 확장합니다.
                // 2. 자신을 두 번 더하면 곱하기 2 (시프트 연산과 동일)가 됩니다.
                // 3. Sigmoid의 1.0스케일이 128이므로 0.5스케일은 64입니다. 정수 64를 빼줍니다.
                s1_tx_adj <= {8'd0, i_tx} + {8'd0, i_tx} - 16'd64; 
                s1_ty_adj <= {8'd0, i_ty} + {8'd0, i_ty} - 16'd64;
                
                // [W, H 방정식 풀이 시작]: YOLOv5 공식 = (2 * sigmoid)^2 * anchor
                // 크기는 빼기(0.5)가 없으므로 2배로 확장만 해둡니다.
                s1_tw_x2  <= {8'd0, i_tw} + {8'd0, i_tw};
                s1_th_x2  <= {8'd0, i_th} + {8'd0, i_th};

                // [앵커 선택 MUX]: Splitter에서 넘어온 앵커 인덱스(0,1,2)를 보고
                // Top에서 내려준 진짜 앵커 픽셀 크기(16-bit) 중 하나를 골라서 레지스터에 박습니다.
                case (i_anchor)
                    2'd0: begin s1_pw <= i_pw_0; s1_ph <= i_ph_0; end
                    2'd1: begin s1_pw <= i_pw_1; s1_ph <= i_ph_1; end
                    2'd2: begin s1_pw <= i_pw_2; s1_ph <= i_ph_2; end
                    default: begin s1_pw <= 0; s1_ph <= 0; end
                endcase

                // 수학 계산기가 돌지 않는 나머지 데이터들은 1번 방에 짐을 풉니다. (1클럭 지연)
                s1_cx <= i_cx; s1_cy <= i_cy;
                s1_conf <= i_conf; s1_cls <= i_cls;
            end
        end
    end

    // =========================================================================
    // [Pipeline Stage 2] 좌표 더하기(Add) & 박스 비율 제곱(Square) DSP 투입
    // =========================================================================
    reg signed [15:0] s2_bx_grid, s2_by_grid; // 그리드 좌표가 더해진 중간 결과
    reg [31:0] s2_tw_sq, s2_th_sq;            // 16비트끼리 곱하면 32비트가 되므로 큰 그릇 할당
    
    // Bypass용 지연(Delay) 레지스터 (Stage 2 방)
    reg [15:0] s2_pw, s2_ph;
    reg [DATA_WIDTH-1:0] s2_conf, s2_cls;
    reg s2_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 리셋 시 2번 방 완전 소독
            s2_bx_grid <= 0; s2_by_grid <= 0; s2_tw_sq <= 0; s2_th_sq <= 0;
            s2_pw <= 0; s2_ph <= 0; s2_conf <= 0; s2_cls <= 0; s2_valid <= 0;
        end else if (ready_in) begin // 💡[Freeze 방어] 다음 단이 막히면 2번 방 연산 일시 정지!
            s2_valid <= s1_valid;
            
            if (s1_valid) begin
                // [X, Y 방정식 중간 풀이]: 앞서 구한 값에 현재 내 그리드 좌표(cx)를 더해줍니다.
                // 단, cx는 1, 2 같은 정수이고 계산 중인 s1_tx_adj는 1.0을 128로 치는 스케일 세계에 있으므로,
                // cx에 128을 곱해(왼쪽 7칸 시프트) 같은 단위계를 맞춰준 뒤에 더해줍니다!
                s2_bx_grid <= s1_tx_adj + {8'd0, s1_cx[7:0]} * 16'd128;
                s2_by_grid <= s1_ty_adj + {8'd0, s1_cy[7:0]} * 16'd128;

                // [W, H 방정식 중간 풀이]: 2*sigmoid 값을 자기 자신과 곱하여 제곱(Square)합니다.
                // Vivado 합성기는 이 코드를 읽고 무조건 FPGA 내부의 귀중한 고속 곱셈기(DSP48E1)로 배선합니다.
                s2_tw_sq <= s1_tw_x2 * s1_tw_x2;
                s2_th_sq <= s1_th_x2 * s1_th_x2;

                // 연산을 마친 앵커 크기와 여전히 꿀 빠는 Bypass 데이터들은 2번째 방으로 옮겨줍니다. (2클럭 지연)
                s2_pw <= s1_pw; s2_ph <= s1_ph;
                s2_conf <= s1_conf; s2_cls <= s1_cls;
            end
        end
    end

    // =========================================================================
    // [조합 회로] 💡 [치명적 버그 수정] 48-bit 곱셈기 명시적 선언 
    // 설명: 16-bit 앵커 크기와 32-bit 제곱값을 곱하면 수학적으로 최대 48-bit까지 팽창합니다.
    // 이를 시프트(>>)하기 전에 컴파일러가 제멋대로 16비트로 잘라버리는 
    // 악랄한 절삭(Truncation) 버그를 막기 위해 48-bit 초대형 와이어를 명시적으로 선언합니다.
    // =========================================================================
    wire signed [47:0] w_bw_mult;
    wire signed [47:0] w_bh_mult;
    
    // 부호 있는(signed) 곱셈임을 명시하여 안전하게 48비트짜리 곱셈 그릇에 담아둡니다. (여기서 DSP 2개가 추가 소모됨)
    assign w_bw_mult = $signed(s2_pw) * $signed(s2_tw_sq);
    assign w_bh_mult = $signed(s2_ph) * $signed(s2_th_sq);
    
    // Q0.7을 제곱하여 14비트로 스케일이 뻥튀기된 것을 원상 복구하기 위해
    // 48비트 와이어를 산술 우측 시프트(>>> 14) 하여 16비트로 깎아냅니다.
    wire signed [47:0] w_bw_shifted = w_bw_mult >>> 14;
    wire signed [47:0] w_bh_shifted = w_bh_mult >>> 14;

    // =========================================================================
    // [Pipeline Stage 3] 최종 스케일 복원 및 출력 패킹
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 리셋 시 출력단 완전 소독
            o_bx <= 0; o_by <= 0; o_bw <= 0; o_bh <= 0;
            o_conf <= 0; o_cls <= 0; o_valid <= 0;
        end else if (ready_in) begin // 💡[Freeze 방어] 이더넷 꽉 차면 배출 금지!
            // 마침내 3개의 스테이지를 모두 통과한 위대한 유효 신호가 바깥 세상으로 나갑니다.
            o_valid <= s2_valid; 
            
            if (s2_valid) begin
                // [X, Y 방정식 최종 완성]: 
                // 곱하기 Stride(8 또는 16) 연산과 나누기 스케일(128) 연산을 한 방에 처리합니다.
                // Top 모듈이 던져준 i_downshift 횟수(P3면 4번, P4면 3번)만큼 
                // 우측 시프트(>>>)하여 깔끔한 16-bit 픽셀 좌표를 뽑아냅니다.
                o_bx <= s2_bx_grid >>> i_downshift;
                o_by <= s2_by_grid >>> i_downshift;

                // [W, H 방정식 최종 완성]:
                // 위 조합 회로에서 시프트 연산을 마친 48비트 와이어의 밑바닥 16비트( [OUT_WIDTH-1:0] )만
                // 안전하게 똑 떼어서 최종 출력 포트에 예쁘게 담습니다.
                o_bw <= w_bw_shifted[OUT_WIDTH-1:0];
                o_bh <= w_bh_shifted[OUT_WIDTH-1:0];

                // [Bypass 데이터 최종 방출]: 
                // X, Y, W, H가 열심히 DSP를 타며 3클럭 동안 계산되는 동안,
                // 옆방에서 조용히 3클럭의 시간을 보내며 타이밍을 완벽하게 맞춘 conf와 cls를 
                // 한날 한시에 똑같이 방출합니다. (Sync 완료!)
                o_conf <= s2_conf;
                o_cls  <= s2_cls;
            end
        end
    end

endmodule