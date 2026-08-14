`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: window_generator_6x6
// Description: 
//   - Layer 0 (Stem) 전용 6x6 윈도우 생성 및 Stride=2 제어 모듈
//   - [Zero-Padding] 논리적 좌표를 계산하여 이미지 밖의 영역은 0으로 채움
//   - [Planar Packing] PyTorch 가중치 추출 배열에 맞춰 R(36)->G(36)->B(36) 순서로 정렬
//   - [Stall 제어] ready_in 신호 연동으로 완벽한 파이프라인 Freeze 지원
//////////////////////////////////////////////////////////////////////////////////

module window_generator_6x6 #(
    parameter DATA_WIDTH = 24, 
    parameter MAX_WIDTH  = 640
)(
    input wire clk, rst_n,
    input wire [10:0] img_width, img_height,

    input wire [DATA_WIDTH-1:0] row0_in, row1_in, row2_in, row3_in, row4_in, row5_in,
    input wire valid_in, ready_in,
    output wire ready_out,

    output reg [863:0] window_out,
    output reg valid_out
);

    // =========================================================================
    // [1] 💡 [수술 완료] Row-Flush FSM (우측 패딩 강제 밀어내기)
    // =========================================================================
    reg [1:0] pad_cnt; // 2개의 추가 시프트를 카운트
    wire is_row_flush = (pad_cnt > 0);
    
    // Row-Flush 중일 때는 앞단(Line Buffer)에 0을 줘서 멈추게 함
    assign ready_out = ready_in && !is_row_flush;

    // 내부 시프트는 valid_in이 들어오거나, Row-Flush 중일 때 동작
    wire shift_en = (valid_in || is_row_flush) && ready_in;

    reg [10:0] px, py;

    always @(posedge clk) begin
        if (!rst_n) begin
            px <= 0; py <= 0; pad_cnt <= 0;
        end else if (ready_in) begin
            if (shift_en) begin
                // W+1 까지 도달하면(추가 2시프트 완료) 롤백
                if (px == img_width + 1) begin
                    px <= 0;
                    pad_cnt <= 0;
                    if (py == img_height + 4) py <= 0;
                    else py <= py + 1;
                end else begin
                    px <= px + 1;
                    // 원본 픽셀 수신이 끝나면 Flush 트리거 온
                    if (px == img_width - 1) pad_cnt <= 1;
                    else if (pad_cnt > 0) pad_cnt <= pad_cnt + 1;
                end
            end
        end
    end

    // =========================================================================
    // [2] 6x6 Shift Register Grid
    // =========================================================================
    reg [DATA_WIDTH-1:0] window_reg [0:5][0:5];
    integer r, c;
    
    initial begin
        for (r = 0; r < 6; r = r + 1) begin
            for (c = 0; c < 6; c = c + 1) begin
                window_reg[r][c] = 0;
            end
        end
    end
    
    always @(posedge clk) begin
        if (shift_en) begin
            for (r = 0; r < 6; r = r + 1) begin
                for (c = 0; c < 5; c = c + 1) begin
                    window_reg[r][c] <= window_reg[r][c+1];
                end
            end
            // 💡 Flush 중에는 0을 삽입하여 우측 패딩을 물리적으로 형성
            window_reg[0][5] <= is_row_flush ? 24'd0 : row0_in;
            window_reg[1][5] <= is_row_flush ? 24'd0 : row1_in;
            window_reg[2][5] <= is_row_flush ? 24'd0 : row2_in;
            window_reg[3][5] <= is_row_flush ? 24'd0 : row3_in;
            window_reg[4][5] <= is_row_flush ? 24'd0 : row4_in;
            window_reg[5][5] <= is_row_flush ? 24'd0 : row5_in; 
        end
    end

    // =========================================================================
    // [3] 동기화된 현재 좌표 (current_x, current_y) 도출
    // =========================================================================
    wire signed [11:0] s_img_width  = {1'b0, img_width};
    wire signed [11:0] s_img_height = {1'b0, img_height};

    // 💡 px가 W+1 까지 돌고 0이 되므로, 0일 때의 이전 좌표는 W+1임!
    wire signed [11:0] current_x = (px == 0) ? (s_img_width + 1)  : ($signed({1'b0, px}) - 1);
    wire signed [11:0] current_y = (px == 0) ? ($signed({1'b0, py}) - 1) : $signed({1'b0, py});

    // =========================================================================
    // [4] 동적 Zero-Padding 및 PyTorch Planar Packing
    // =========================================================================
    wire [863:0] next_window_out;
    wire signed [11:0] x_coords [0:5];
    wire signed [11:0] y_coords [0:5];

    genvar gc, gr;
    generate
        for(gc = 0; gc < 6; gc = gc + 1) begin : X_COORD
            assign x_coords[gc] = current_x - 5 + gc;
        end
        for(gr = 0; gr < 6; gr = gr + 1) begin : Y_COORD
            assign y_coords[gr] = current_y - 5 + gr;
        end
        
        for(gr = 0; gr < 6; gr = gr + 1) begin : ROW_PACK
            for(gc = 0; gc < 6; gc = gc + 1) begin : COL_PACK
                wire x_valid = (x_coords[gc] >= 0) && (x_coords[gc] < s_img_width);
                wire y_valid = (y_coords[gr] >= 0) && (y_coords[gr] < s_img_height);
                
                wire [23:0] padded_pixel = (x_valid && y_valid) ? window_reg[gr][gc] : 24'd0;
                
                wire [7:0] r_val = padded_pixel[7:0];
                wire [7:0] g_val = padded_pixel[15:8];
                wire [7:0] b_val = padded_pixel[23:16];

                localparam s_idx = gr * 6 + gc;
                assign next_window_out[(0 * 36 + s_idx) * 8 +: 8] = r_val;
                assign next_window_out[(1 * 36 + s_idx) * 8 +: 8] = g_val;
                assign next_window_out[(2 * 36 + s_idx) * 8 +: 8] = b_val;
            end
        end
    endgenerate

    // =========================================================================
    // [5] PyTorch Stride=2 제어 및 출력
    // =========================================================================
    wire signed [11:0] top_left_x = current_x - 5;
    wire signed [11:0] top_left_y = current_y - 5;
    
    wire x_valid_stride = (top_left_x >= -2) && (top_left_x <= s_img_width - 4)  && (top_left_x[0] == 1'b0);
    wire y_valid_stride = (top_left_y >= -2) && (top_left_y <= s_img_height - 4) && (top_left_y[0] == 1'b0);

    reg valid_s1;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 0;
            window_out <= 0;
            valid_s1 <= 0;
        end else if (ready_in) begin
            // 💡 Flush 시프트 중에도 5번째 윈도우를 출력해야 하므로 shift_en 사용!
            valid_s1 <= shift_en;
            if (valid_s1) begin
                valid_out <= x_valid_stride && y_valid_stride;
                window_out <= next_window_out;
            end else begin
                valid_out <= 0;
            end
        end
    end
endmodule