`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: weight_rom_array  - DEPTH 재산정판 (best.pt 실측 기준)
// Description:
//   - 3x3 엔진용 가중치(18,432-bit)와 1x1 엔진용 가중치(2,048-bit)를 독립 BRAM 에 저장.
//   - [정밀 재단] best.pt 실측 추출 라인 수:
//       1x1 weight 라인 = 605  -> W1_DEPTH = 1024 (2의 거듭제곱, 여유)
//       3x3 weight 라인 = 104  -> W3_DEPTH = 128  (stem 6x6 포함; 6x6 은 그룹당 1라인)
//   - ready_in=0 시 메모리 포인터 동결(Stall).  주소(waddr)는 FSM 의 weight_addr +
//     (그룹*입력폴드 + 폴드) 누적 오프셋으로, 상위(컨트롤러)에서 증가시켜 인가한다.
//   - 6x6 stem: 3x3 엔진 재사용(is_layer_0). 가중치도 3x3 ROM(rom_3x3)에 18432-bit 1라인
//     (144 폭에 108 채우고 제로패딩)으로 저장된다.
//////////////////////////////////////////////////////////////////////////////////

module weight_rom_array #(
    parameter W3_WIDTH   = 18432,
    parameter W1_WIDTH   = 2048,
    parameter ADDR_WIDTH = 14,
    parameter W3_DEPTH   = 128,    // 실측 104 (3x3+stem) 수용
    parameter W1_DEPTH   = 1024    // 실측 605 수용
)(
    input wire clk,
    input wire ready_in,
    input wire [ADDR_WIDTH-1:0] waddr,
    output reg [W3_WIDTH-1:0] weight_3x3_out = 0,
    output reg [W1_WIDTH-1:0] weight_1x1_out = 0
);
    (* ram_style = "block" *) reg [W3_WIDTH-1:0] rom_3x3 [0:W3_DEPTH-1];
    (* ram_style = "block" *) reg [W1_WIDTH-1:0] rom_1x1 [0:W1_DEPTH-1];

    integer i;
    initial begin
        for (i=0;i<W3_DEPTH;i=i+1) rom_3x3[i]=0;
        for (i=0;i<W1_DEPTH;i=i+1) rom_1x1[i]=0;
        $readmemh("weights_3x3.mem", rom_3x3);
        $readmemh("weights_1x1.mem", rom_1x1);
    end

    always @(posedge clk) begin
        if (ready_in) begin
            weight_3x3_out <= rom_3x3[waddr];
            weight_1x1_out <= rom_1x1[waddr];
        end
    end
endmodule
