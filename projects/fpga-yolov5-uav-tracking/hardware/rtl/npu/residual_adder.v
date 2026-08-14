`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: residual_adder
// Description: 
//   - C3 Bottleneck의 Shortcut 연산을 위한 16채널 병렬 덧셈기 (Verilog-2001 완벽 준수)
//   - [문법 오류 수정] always 블록 내부의 로컬 변수 선언(SystemVerilog 문법)을 제거하고,
//     generate 블록 기반의 Combinational Wire 매핑으로 변경하여 문법적 무결성을 확보함.
//   - [8-bit Saturation] INT8 + INT8 = 9-bit 결과를 오버/언더플로우 없이 안전하게 INT8로 클리핑.
//////////////////////////////////////////////////////////////////////////////////

module residual_adder #(
    parameter T0 = 16,           // 병렬 채널 수
    parameter DATA_WIDTH = 8     // 데이터 비트 폭 (INT8)
)(
    input wire clk,
    input wire rst_n,

    // [Input 1] 메인 데이터 (m0_cv2 결과)
    input wire [T0*DATA_WIDTH-1:0] data_main_in, 
    input wire                     valid_in,
    input wire                     last_fold_in,
    // (디버깅 및 메타데이터용 사이드밴드 신호)
    input wire [10:0]              px_in,
    input wire [10:0]              py_in,
    
    // [Input 2] 잔차 데이터 (GFB에서 꺼내온 cv1 결과)
    input wire [T0*DATA_WIDTH-1:0] data_res_in,  
    
    input wire ready_in,   // 뒷단의 Stall 신호
    output wire ready_out, // 앞단으로 전달할 Stall 신호

    // [Output] 클리핑이 완료된 데이터 (write_packer로 직행)
    output reg [T0*DATA_WIDTH-1:0] sum_data_out, 
    output reg                     valid_out,
    output reg                     last_fold_out,
    output reg [10:0]              px_out,
    output reg [10:0]              py_out
);

    assign ready_out = ready_in;

    // =========================================================================
    // [1] Combinational Logic: 16채널 병렬 덧셈 및 클리핑 (generate 구문 활용)
    // =========================================================================
    wire [T0*DATA_WIDTH-1:0] w_clipped_out;

    genvar i;
    generate
        for (i = 0; i < T0; i = i + 1) begin : ADDER_CORE
            // 1. 거대 버스에서 개별 채널의 8-bit 데이터 추출
            wire signed [DATA_WIDTH-1:0] val_main = data_main_in[i*DATA_WIDTH +: DATA_WIDTH];
            wire signed [DATA_WIDTH-1:0] val_res  = data_res_in[i*DATA_WIDTH +: DATA_WIDTH];
            
            // 2. 부호 확장을 동반한 9-bit 안전 덧셈
            wire signed [8:0] temp_sum = val_main + val_res;

            // 3. 삼항 연산자를 이용한 8-bit 클리핑 (Verilog-2001 표준)
            assign w_clipped_out[i*DATA_WIDTH +: DATA_WIDTH] = 
                (temp_sum > 9'sd127)  ? 8'sd127 :  // 오버플로우 시 +127 고정
                (temp_sum < -9'sd128) ? -8'sd128 : // 언더플로우 시 -128 고정
                                        temp_sum[7:0];     // 안전 범위는 하위 8비트 취함
        end
    endgenerate

    // =========================================================================
    // [2] Sequential Logic: 1-Clock Pipeline Register & Stall Control
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            sum_data_out  <= 0;
            valid_out     <= 0;
            last_fold_out <= 0;
            px_out        <= 0;
            py_out        <= 0;
        end 
        else if (ready_in) begin
            // 💡 덧셈이 완료된 데이터와 메타데이터(valid, last_fold, px, py)를
            // 정확히 동일하게 1클럭 지연시켜 출력합니다.
            sum_data_out  <= w_clipped_out;
            valid_out     <= valid_in;
            last_fold_out <= last_fold_in;
            px_out        <= px_in;
            py_out        <= py_in;
        end
    end

endmodule