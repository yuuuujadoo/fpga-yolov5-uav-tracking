`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: activation_silu
// Description: 
//   - SiLU LUT 코어를 T0(16)개 병렬로 인스턴스화(복제)하는 래퍼(Wrapper) 모듈
//   - Re-quantization을 거친 16-bit 입력을 받아 8-bit 최종 양자화 데이터로 병렬 출력
//   - [추가] 외부 ready_in 신호를 16개의 BRAM 모듈에 브로드캐스팅하여 동시 정지 제어
//////////////////////////////////////////////////////////////////////////////////

module activation_silu #(
    parameter T0 = 16,             // 병렬 연산 채널 수
    parameter DATA_WIDTH = 8       // INT8 데이터 폭
)(
    input  wire clk, rst_n,
    
    input  wire [T0*DATA_WIDTH-1:0] requant_data_in, // 16채널 주소 버스
    input  wire valid_in,                            // 데이터 유효 깃발
    input  wire ready_in,                            // 💡 [추가] 뒷단 멈춤 신호
    
    output wire ready_out,                           // 💡 [추가] 앞단 멈춤 신호
    output wire [T0*DATA_WIDTH-1:0] silu_data_out,   // 16채널 활성화 결과 버스
    output wire valid_out                            // 결과 유효 깃발
);

    wire [DATA_WIDTH-1:0] w_in_data  [0:T0-1];
    wire [DATA_WIDTH-1:0] w_out_data [0:T0-1];
    wire [T0-1:0] w_silu_valid;
    wire [T0-1:0] w_ready_out;

    assign valid_out = w_silu_valid[0];
    assign ready_out = w_ready_out[0];

    genvar i;
    // [Unpacking] 거대 버스를 8비트씩 자르기
    generate
        for (i = 0; i < T0; i = i + 1) begin : UNPACK
            assign w_in_data[i] = requant_data_in[(i * DATA_WIDTH) +: DATA_WIDTH];
        end
    endgenerate
    
    // [LUT 코어 인스턴스화]
    generate
        for (i = 0; i < T0; i = i + 1) begin : SILU_ARRAY
            silu_lut_unit #(
                .DATA_WIDTH(DATA_WIDTH)
            ) u_silu (
                .clk(clk), .rst_n(rst_n),
                .din(w_in_data[i]), .valid_in(valid_in),
                .ready_in(ready_in), // 💡 16개 메모리에 일제히 브레이크 하달
                .ready_out(w_ready_out[i]),
                .dout(w_out_data[i]), .valid_out(w_silu_valid[i])
            );
        end
    endgenerate
    
    // [Packing] 8비트 결과들을 다시 거대 버스로 조립
    generate
        for (i = 0; i < T0; i = i + 1) begin : PACK
            assign silu_data_out[(i * DATA_WIDTH) +: DATA_WIDTH] = w_out_data[i];
        end
    endgenerate

endmodule