`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: global_feature_buffer
// Description: 
//   - [2D 물리적 슬라이싱 적용] Width Slicing(16분할) + Depth Slicing(8192+1408)
//   - Vivado가 주소 폭(14-bit)을 보고 BRAM을 16384 깊이로 멍청하게 뻥튀기하는 
//     현상을 원천 차단하기 위해, 메모리 배열을 물리적으로 두 동강 내어 선언했습니다.
//   - 최종적으로 1024 타일 -> 640 타일로 BRAM 소모를 강제 압축합니다.
//   - (외부 인터페이스는 기존 2048-bit 구조를 100% 동일하게 유지합니다)
//////////////////////////////////////////////////////////////////////////////////

module global_feature_buffer #(
    parameter DATA_WIDTH = 2048, 
    parameter DEPTH      = 9600, // 정밀 재단된 깊이
    parameter ADDR_WIDTH = 14    
)(
    input wire clk,

    input wire [DATA_WIDTH-1:0] wdata,
    input wire [ADDR_WIDTH-1:0] waddr,
    input wire                  we,

    input wire [ADDR_WIDTH-1:0] raddr,
    input wire                  re,
    output wire [DATA_WIDTH-1:0] rdata
);

    localparam SLICE_WIDTH = 128; 
    localparam NUM_SLICES  = DATA_WIDTH / SLICE_WIDTH; 
    
    localparam DEPTH_LOWER = 8192;                     
    localparam DEPTH_UPPER = DEPTH - DEPTH_LOWER;      

    wire [SLICE_WIDTH-1:0] w_rdata_slices [0:NUM_SLICES-1];

    genvar i;
    generate
        for (i = 0; i < NUM_SLICES; i = i + 1) begin : BRAM_2D_SLICES
            // Vivado가 개별적으로 정밀하게 합성하도록 BRAM을 두 동강 내어 선언
            (* ram_style = "block" *) reg [SLICE_WIDTH-1:0] ram_lower [0:DEPTH_LOWER-1]; 
            (* ram_style = "block" *) reg [SLICE_WIDTH-1:0] ram_upper [0:DEPTH_UPPER-1]; 
            
            // 시뮬레이터 X 전파 차단
            // synthesis translate_off
            integer j;
            initial begin
                for (j = 0; j < DEPTH_LOWER; j = j + 1) ram_lower[j] = 0;
                for (j = 0; j < DEPTH_UPPER; j = j + 1) ram_upper[j] = 0;
            end
            // synthesis translate_on

            reg [SLICE_WIDTH-1:0] rdata_lower = 0;
            reg [SLICE_WIDTH-1:0] rdata_upper = 0;
            reg                   raddr_is_upper_reg = 0; 

            always @(posedge clk) begin
                // 쓰기 동작 (Bound Check 적용)
                if (we) begin
                    if (waddr < DEPTH_LOWER) begin
                        ram_lower[waddr[12:0]] <= wdata[i*SLICE_WIDTH +: SLICE_WIDTH];
                    end 
                    else if (waddr < DEPTH) begin
                        ram_upper[waddr - DEPTH_LOWER] <= wdata[i*SLICE_WIDTH +: SLICE_WIDTH];
                    end
                end
                
                // 읽기 동작 (1클럭 BRAM 레이턴시)
                if (re) begin
                    raddr_is_upper_reg <= (raddr >= DEPTH_LOWER);
                    
                    if (raddr < DEPTH_LOWER) begin
                        rdata_lower <= ram_lower[raddr[12:0]];
                    end 
                    else if (raddr < DEPTH) begin
                        rdata_upper <= ram_upper[raddr - DEPTH_LOWER];
                    end
                end
            end

            // 상단/하단 뱅크 중 올바른 값을 선택하여 2D 와이어 배열에 매핑
            assign w_rdata_slices[i] = raddr_is_upper_reg ? rdata_upper : rdata_lower;
            
        end
    endgenerate

    // 16개의 조각을 강제 순서 배정하여 시뮬레이터 호환성(Endianness) 완벽 보장
    assign rdata = {
        w_rdata_slices[15], w_rdata_slices[14], w_rdata_slices[13], w_rdata_slices[12],
        w_rdata_slices[11], w_rdata_slices[10], w_rdata_slices[9],  w_rdata_slices[8],
        w_rdata_slices[7],  w_rdata_slices[6],  w_rdata_slices[5],  w_rdata_slices[4],
        w_rdata_slices[3],  w_rdata_slices[2],  w_rdata_slices[1],  w_rdata_slices[0]
    };

endmodule