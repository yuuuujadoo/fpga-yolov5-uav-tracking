`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: param_rom_array  (bias + M0 + n 통합 per-channel ROM)
// Description:
//   - [per-channel 양자화 채택에 따른 신규 통합 ROM]
//     기존 bias_rom_array 를 확장하여, 출력채널 16개 그룹당 한 줄(640-bit)에
//     { n[16ch x 8b] | M0[16ch x 16b] | bias[16ch x 16b] } 를 함께 저장한다.
//   - per-channel M0/n 은 그룹당 16개라 128-bit 마이크로 인스트럭션(parameter_rom)에
//     담을 수 없으므로, weight 처럼 그룹 단위로 스트리밍하는 별도 ROM 으로 분리한다.
//   - 1x1 엔진용/3x3 엔진용을 독립 BRAM 으로 분리(정밀 재단). FSM 의 param_addr(그룹 base)
//     를 주소로 1클럭 만에 한 그룹의 bias/M0/n 을 동시 출력 -> conv(bias) & post_processor(M0,n).
//
//   라인 비트맵 (MSB=639):
//     [639:512] n_packed   : ch15..ch0, 각 8-bit (5-bit n 을 8-bit 정렬)  = 128b
//     [511:256] M0_packed  : ch15..ch0, 각 signed 16-bit                  = 256b
//     [255:  0] bias_packed: ch15..ch0, 각 signed 16-bit                  = 256b
//   => post_processor 의 M0_in[T0*16], n_in[T0*5] 및 conv 의 bias_in[T0*16] 으로 분배.
//      (n 은 하위 5-bit 만 사용; 상위 3-bit 는 0)
//////////////////////////////////////////////////////////////////////////////////

module param_rom_array #(
    parameter LINE_WIDTH = 640,      // n(128) + M0(256) + bias(256)
    parameter ADDR_WIDTH = 10,       // 그룹 단위 주소

    // 💡 [정밀 재단] 실제 best.pt 추출 라인 수 기준 (1x1:87, 3x3:34) + 여유
    parameter P1_DEPTH = 128,        // 1x1 그룹 라인 (실제 87)
    parameter P3_DEPTH = 64          // 3x3 그룹 라인 (실제 34, stem 포함)
)(
    input  wire clk,
    input  wire ready_in,                       // 파이프라인 정지(Stall) 시 포인터 동결
    input  wire [ADDR_WIDTH-1:0] paddr,         // FSM 의 param_addr (그룹 base + 그룹오프셋)

    // 분배 출력 (1x1/3x3 동시 출력; FSM 의 is_1x1 로 상위에서 선택)
    output reg  [255:0] bias_1x1_out = 0,
    output reg  [255:0] M0_1x1_out   = 0,
    output reg  [127:0] n_1x1_out    = 0,
    output reg  [255:0] bias_3x3_out = 0,
    output reg  [255:0] M0_3x3_out   = 0,
    output reg  [127:0] n_3x3_out    = 0
);
    (* ram_style = "block" *) reg [LINE_WIDTH-1:0] rom_1x1 [0:P1_DEPTH-1];
    (* ram_style = "block" *) reg [LINE_WIDTH-1:0] rom_3x3 [0:P3_DEPTH-1];

    integer i;
    initial begin
        for (i=0;i<P1_DEPTH;i=i+1) rom_1x1[i]=0;
        for (i=0;i<P3_DEPTH;i=i+1) rom_3x3[i]=0;
        $readmemh("param_1x1.mem", rom_1x1);
        $readmemh("param_3x3.mem", rom_3x3);
    end

    reg [LINE_WIDTH-1:0] line1, line3;
    always @(posedge clk) begin
        if (ready_in) begin
            line1 <= rom_1x1[paddr];
            line3 <= rom_3x3[paddr];
        end
    end

    // 비트 슬라이싱 분배 (조합)
    always @(*) begin
        bias_1x1_out = line1[255:0];
        M0_1x1_out   = line1[511:256];
        n_1x1_out    = line1[639:512];
        bias_3x3_out = line3[255:0];
        M0_3x3_out   = line3[511:256];
        n_3x3_out    = line3[639:512];
    end
endmodule
