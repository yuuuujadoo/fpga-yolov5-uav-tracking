`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: requantization
// Description: 
//   - Conv 모듈의 32-bit Psum을 받아 8-bit SiLU 주소로 압축(Scale & Shift)하는 모듈
//   - [8-bit 다이어트 적용 완료] OUT_WIDTH가 8로 축소되었으며, 
//     Stage 3에서 -128 ~ 127 범위를 벗어나는 데이터에 대한 강력한 Saturation(클리핑) 적용
//   - 3-Stage Pipeline: 곱셈(48-bit) -> 반올림/시프트 -> 클리핑(8-bit)
//////////////////////////////////////////////////////////////////////////////////

module requantization #(
    // [파라미터 선언] 데이터 비트 폭 정의
    parameter PSUM_WIDTH = 32,   // Conv에서 들어오는 32비트 누적합
    parameter M0_WIDTH   = 16,   // 스케일링을 위한 16비트 곱셈 계수
    parameter OUT_WIDTH  = 8     // 최종 출력되는 8비트 데이터
)(
    input wire clk, rst_n,       // 메인 클럭 및 비동기 리셋

    // [설정 값 입력] (동적으로 변할 수 있는 양자화 파라미터)
    input wire signed [M0_WIDTH-1:0] M0_in, // 곱셈 계수
    input wire [4:0]                 n_in,  // 비트 이동(Shift) 횟수

    // [데이터 및 제어 입력]
    input wire signed [PSUM_WIDTH-1:0] psum_in, // 32비트 연산 결과
    input wire valid_in,                        // 데이터 유효 깃발
    input wire ready_in,                        // 💡 [추가] 뒷단이 바쁠 때 보내는 멈춤 신호

    // [데이터 및 제어 출력]
    output wire ready_out,                      // 💡 [추가] 앞단으로 보내는 나의 멈춤 상태
    output reg signed [OUT_WIDTH-1:0] requant_out, // 8비트 최종 양자화 데이터
    output reg valid_out                        // 출력 데이터 유효 깃발
);

    // 내부 파이프라인만 존재하므로, 뒷단의 멈춤 신호를 앞단으로 직결하여 투명하게 전달합니다.
    assign ready_out = ready_in;

    // =========================================================================
    // [Pipeline Stage 1] 48-bit 곱셈 연산
    // 설명: 32-bit Psum과 16-bit M0를 곱하여 48-bit 결과를 만듭니다. (1클럭 지연)
    // =========================================================================
    reg signed [47:0] mult_reg; // 곱셈 결과를 담을 48비트 레지스터
    reg [4:0]         n_s1;     // Stage 2에서 쓸 Shift 값을 릴레이하기 위한 레지스터
    reg               valid_s1; // Stage 2로 넘길 유효 깃발

    always @(posedge clk) begin
        if (!rst_n) begin
            mult_reg <= 0; n_s1 <= 0; valid_s1 <= 0;
        end 
        else if (ready_in) begin // 💡 [Stall 방어] 멈춤 신호가 오면 이전 값을 유지(Freeze)
            valid_s1 <= valid_in;
            if (valid_in) begin
                // 부호 있는 곱셈 수행 및 Shift 파라미터 전달
                mult_reg <= $signed(psum_in) * $signed(M0_in);
                n_s1     <= n_in;
            end
        end
    end

    // =========================================================================
    // [Pipeline Stage 2] 반올림(Rounding) 및 산술 시프트(Arithmetic Shift)
    // 설명: 시프트로 날아가는 비트를 보정하기 위해 절반값(half_val)을 더하고 시프트합니다.
    // =========================================================================
    // 조합 회로(@*): 시프트 값(n_s1)이 0보다 크면 절반 위치의 비트를 1로 만듭니다.
    wire signed [47:0] half_val = (n_s1 > 0) ? (48'sd1 << (n_s1 - 1)) : 48'sd0;
    wire signed [47:0] pre_shift_val = mult_reg + half_val; // 반올림 덧셈
    
    reg signed [47:0] shift_reg; // 시프트 결과를 담을 레지스터
    reg               valid_s2;  // Stage 3로 넘길 유효 깃발

    always @(posedge clk) begin
        if (!rst_n) begin
            shift_reg <= 0; valid_s2 <= 0;
        end 
        else if (ready_in) begin // 💡 [Stall 방어]
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                // 부호를 유지하며 오른쪽으로 미는 산술 시프트(>>>) 수행
                shift_reg <= pre_shift_val >>> n_s1; 
            end
        end
    end

    // =========================================================================
    // [Pipeline Stage 3] 8-bit 클리핑(Saturation) 및 출력
    // 설명: -128 ~ 127의 범위를 벗어나면 최대/최소값으로 깎아냅니다(Clamp).
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            requant_out <= 0; valid_out <= 0;
        end 
        else if (ready_in) begin // 💡 [Stall 방어]
            valid_out <= valid_s2;
            if (valid_s2) begin
                // INT8 상한선 초과 시 127로 고정
                if (shift_reg > 127) begin
                    requant_out <= 8'sd127;
                end 
                // INT8 하한선 미달 시 -128로 고정
                else if (shift_reg < -128) begin
                    requant_out <= -8'sd128;
                end 
                // 정상 범위면 하위 8비트만 안전하게 추출
                else begin
                    requant_out <= shift_reg[7:0];
                end
            end
        end
    end
endmodule