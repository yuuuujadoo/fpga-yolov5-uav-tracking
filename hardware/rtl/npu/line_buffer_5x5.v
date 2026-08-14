`timescale 1ns / 1ps

// =================================================================================
// 모듈명: line_buffer_5x5
// 설명: 
//   - 128-bit 스트림을 받아 5줄(Row)로 폭포수처럼 엮어주는 BRAM 버퍼.
//   - 💡 [좌표 패스스루 수술] 앞단(Channel Folder)에서 만들어진 px, py 좌표를 받아
//     픽셀 데이터가 1클럭 지연될 때 좌표도 똑같이 1클럭 지연시켜 위상을 맞춥니다.
// =================================================================================

module line_buffer_5x5 #(
    parameter DATA_WIDTH = 8,
    parameter FOLD_CH    = 16,
    parameter MAX_WIDTH  = 20,
    parameter MAX_CH     = 256
)(
    input wire clk, rst_n,
    input wire [10:0] img_width,
    input wire [9:0] total_ch_in,

    input wire [DATA_WIDTH*FOLD_CH-1:0] pixel_in,
    input wire valid_in, last_fold_in,
    input wire [10:0] px_in, py_in, 
    input wire emit_in,
    input wire ready_in,     

    output wire ready_out,   
    output reg [DATA_WIDTH*FOLD_CH-1:0] row0_out, row1_out, row2_out, row3_out, row4_out,
    output reg valid_out, last_fold_out,
    output reg [10:0] px_out, py_out,
    output reg emit_out  
);

    assign ready_out = ready_in;

    (* ram_style = "block" *) reg [DATA_WIDTH*FOLD_CH-1:0] ram0 [0:(MAX_WIDTH+2)*(MAX_CH/16)-1];
    (* ram_style = "block" *) reg [DATA_WIDTH*FOLD_CH-1:0] ram1 [0:(MAX_WIDTH+2)*(MAX_CH/16)-1];
    (* ram_style = "block" *) reg [DATA_WIDTH*FOLD_CH-1:0] ram2 [0:(MAX_WIDTH+2)*(MAX_CH/16)-1];
    (* ram_style = "block" *) reg [DATA_WIDTH*FOLD_CH-1:0] ram3 [0:(MAX_WIDTH+2)*(MAX_CH/16)-1];

    // BRAM X값 전파 차단을 위한 초기화
    integer i;
    initial begin
        for (i=0; i<(MAX_WIDTH+2)*(MAX_CH/16); i=i+1) begin
            ram0[i] = 0; ram1[i] = 0; ram2[i] = 0; ram3[i] = 0;
        end
    end

    reg [15:0] ptr_1d, ptr_d1;    
    reg valid_d1, last_fold_d1;
    reg [DATA_WIDTH*FOLD_CH-1:0] pixel_in_d1;
    
    // 💡 [핵심 수술] 좌표를 1클럭 붙잡아둘 대기실(Pipeline Register) 선언
    reg [10:0] px_d1, py_d1; 
    reg emit_d1;

    // ★ maxpool 격자확장: coord_gen 이 (W+2)x(H+2) 격자를 순회하므로
    //   ptr(열주소) 순환범위도 격자너비(W+2) 기준이어야 함.
    //   (W 기준이면 px=W,W+1 데이터가 ptr 되감겨 col0,1 덮어씀 -> 행/열 섞임)
    wire [15:0] grid_w  = img_width + 2;
    wire [15:0] max_ptr = grid_w * (total_ch_in / FOLD_CH) - 1;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 0; last_fold_out <= 0;
            ptr_1d <= 0; ptr_d1 <= 0;
            valid_d1 <= 0; last_fold_d1 <= 0; pixel_in_d1 <= 0;
            px_d1 <= 0; py_d1 <= 0; // 초기화
            px_out <= 0; py_out <= 0;
            emit_d1 <= 0; emit_out <= 0;
            row0_out <= 0; row1_out <= 0; row2_out <= 0; row3_out <= 0; row4_out <= 0;
        end 
        else if (ready_in) begin 
            
            // -------------------------------------------------------------
            // Phase 1: 1클럭 지연 (대기실 입장)
            // -------------------------------------------------------------
            ptr_d1 <= ptr_1d;
            valid_d1 <= valid_in;
            last_fold_d1 <= last_fold_in;
            pixel_in_d1 <= pixel_in;
            
            px_d1 <= px_in; // 💡 좌표도 대기실에 함께 입장시킴
            py_d1 <= py_in;
            emit_d1 <= emit_in;

            // -------------------------------------------------------------
            // Phase 2: 출력단 래치 (최종 방출)
            // -------------------------------------------------------------
            valid_out <= valid_d1;
            last_fold_out <= last_fold_d1;
            
            // 💡 대기실에 있던 좌표를 데이터와 한날한시에 방출함 (퍼펙트 동기화)
            px_out <= px_d1; 
            py_out <= py_d1;
            emit_out <= emit_d1;
            
            if (valid_d1) begin
                row0_out <= ram3[ptr_d1]; 
                row1_out <= ram2[ptr_d1];
                row2_out <= ram1[ptr_d1];
                row3_out <= ram0[ptr_d1];
                row4_out <= pixel_in_d1;  
            end else begin
                valid_out <= 0;
            end

            // -------------------------------------------------------------
            // Phase 3: 완벽한 수직 하강(Read-First) Cascade BRAM 쓰기
            // -------------------------------------------------------------
            if (valid_d1) begin
                ram3[ptr_d1] <= ram2[ptr_d1];
                ram2[ptr_d1] <= ram1[ptr_d1];
                ram1[ptr_d1] <= ram0[ptr_d1];
                ram0[ptr_d1] <= pixel_in_d1;
            end
            
            if (valid_in) begin
                ptr_1d <= (ptr_1d == max_ptr) ? 0 : ptr_1d + 1;
            end
        end
    end
endmodule