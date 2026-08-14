`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: parameter_rom_array (128-bit Micro-Instruction Decoder) - 확장판
// Description:
//   - 파이썬 메모리 컴파일러가 추출한 128-bit layer_params.mem 을 읽어 1클럭 만에
//     레이어 제어신호를 분배한다.
//   - [신규 확장]
//     * out_ch(10b) 추가  : 출력 채널 수(그룹 루프 횟수 = out_ch/16 산정)
//     * img_width(11b) 추가: 3x3/maxpool 의 line buffer/coord 에 필수
//     * M0(16b)/n(5b) 제거 : per-channel 채택으로 param_rom_array(별도 ROM)로 이관
//     * param_addr(10b)    : 기존 bias_addr 자리를 'bias+M0+n 합본 ROM 의 그룹 base'로 재정의
//   - 유도신호(인스트럭션에 미포함, 상위에서 계산):
//       img_height = ceil(target_pixel / img_width)
//       silu_en    = ~is_detect        (Detect 레이어만 SiLU bypass)
//       in_place   = (waddr == raddr_A) (출력이 입력을 덮어쓰는 in-place 스텝)
//
//   128-bit 비트맵 (instr_format.py 와 1:1 동기화):
//     [125:124] stride      [123] is_detect
//     [122:113] total_ch_in [112:103] out_ch
//     [102:83]  target_pixel
//     [82:69] raddr_A  [68:55] raddr_B  [54:41] waddr
//     [40:27] weight_addr   [26:17] param_addr   [16:6] img_width
//     [5] is_upsample [4] is_concat [3] is_byp [2] is_pool [1] is_layer_0 [0] is_1x1
//////////////////////////////////////////////////////////////////////////////////

module parameter_rom_array #(
    parameter ROM_DEPTH  = 128,   // 최대 레이어 실행 스텝 수
    parameter ADDR_WIDTH = 7,
    parameter DATA_WIDTH = 128
)(
    input  wire                  clk,
    input  wire                  en,
    input  wire [ADDR_WIDTH-1:0] layer_id,

    output wire [1:0]            stride_out,
    output wire                  is_detect_out,
    output wire                  is_p4_out,         // [126] Detect P4 구분
    output wire [9:0]            total_ch_in_out,
    output wire [9:0]            out_ch_out,        // 💡 신규
    output wire [19:0]           target_pixel_out,
    output wire [13:0]           raddr_A_out,
    output wire [13:0]           raddr_B_out,
    output wire [13:0]           waddr_out,
    output wire [13:0]           weight_addr_out,
    output wire [9:0]            param_addr_out,    // 💡 bias+M0+n 합본 ROM base
    output wire [10:0]           img_width_out,     // 💡 신규
    output wire                  is_upsample_out,
    output wire                  is_concat_out,
    output wire                  is_byp_out,
    output wire                  is_pool_out,
    output wire                  is_layer_0_out,
    output wire                  is_1x1_out,

    // 유도 신호 (편의 제공)
    output wire                  silu_en_out,       // = ~is_detect
    output wire                  in_place_out       // = (waddr == raddr_A)
);
    reg [DATA_WIDTH-1:0] param_rom [0:ROM_DEPTH-1];
    integer i;
    initial begin
        for (i=0;i<ROM_DEPTH;i=i+1) param_rom[i]=128'd0;
        $readmemh("layer_params.mem", param_rom);
    end

    reg [DATA_WIDTH-1:0] d=0;
    always @(posedge clk) if (en) d <= param_rom[layer_id];

    assign stride_out       = d[125:124];
    assign is_detect_out    = d[123];
    assign is_p4_out        = d[126];
    assign total_ch_in_out  = d[122:113];
    assign out_ch_out       = d[112:103];
    assign target_pixel_out = d[102:83];
    assign raddr_A_out      = d[82:69];
    assign raddr_B_out      = d[68:55];
    assign waddr_out        = d[54:41];
    assign weight_addr_out  = d[40:27];
    assign param_addr_out   = d[26:17];
    assign img_width_out    = d[16:6];
    assign is_upsample_out  = d[5];
    assign is_concat_out    = d[4];
    assign is_byp_out       = d[3];
    assign is_pool_out      = d[2];
    assign is_layer_0_out   = d[1];
    assign is_1x1_out       = d[0];

    assign silu_en_out  = ~d[123];                 // ~is_detect
    assign in_place_out = (d[54:41] == d[82:69]);  // waddr == raddr_A
endmodule
