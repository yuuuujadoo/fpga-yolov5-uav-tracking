`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: conv_block_1x1 (Pure MAC Core)
// Description: 
//   - Channel Folder와 1x1 Conv Array까지만 포함하는 순수 합성곱 코어.
//   - [자원 최적화] Requantization 및 SiLU 모듈 제거 (Top Datapath에서 3x3과 공유)
//   - [타이밍 동기화] Channel Folder와 Conv 배열 입력 사이 1-Clock Delay Register 적용 완료.
//////////////////////////////////////////////////////////////////////////////////

module conv_block_1x1 #(
    parameter DATA_WIDTH = 8,   
    parameter BIAS_WIDTH = 16,  
    parameter PSUM_WIDTH = 32,  
    parameter MAX_CH = 256,     
    parameter T0 = 16,          
    parameter FOLD_CH = 16,
    parameter FIDX_W = 4        // = clog2(MAX_CH/FOLD_CH), V2001(파라미터화)
)(
    input wire clk,
    input wire rst_n,

    input wire [9:0]                       total_ch_in, 
    input wire [DATA_WIDTH*MAX_CH-1:0]     pixel_in,
    input wire                             valid_in,
    
    input wire [T0*DATA_WIDTH*FOLD_CH-1:0] weight_in,
    input wire [T0*BIAS_WIDTH-1:0]         bias_in,

    output wire [T0*PSUM_WIDTH-1:0]        psum_out,
    output wire                            valid_out,

    input wire                             ready_in,        
    output wire                            ready_out,      
    output wire [FIDX_W:0] fold_idx_out 
);

    wire [DATA_WIDTH*FOLD_CH-1:0] cf_folded_pixel;
    wire cf_valid, cf_last_fold;

    // =========================================================================
    // 1. 1x1 전용 채널 폴더
    // =========================================================================
    channel_folder_1x1 #(
        .DATA_WIDTH(DATA_WIDTH), .MAX_CH(MAX_CH), .FOLD_CH(FOLD_CH)
    ) u_channel_folder_1x1 (
        .clk(clk), .rst_n(rst_n), .total_ch_in(total_ch_in),
        .pixel_in(pixel_in), .valid_in(valid_in),
        .ready_in(ready_in), .ready_out(ready_out),  
        .folded_pixel_out(cf_folded_pixel), .valid_out(cf_valid),
        .last_fold_out(cf_last_fold), .fold_idx(fold_idx_out) 
    );

    // =========================================================================
    // 💡 1.5. Bias Input Latch (편향 입력 래치)
    // 채널 폴더가 새로운 픽셀을 받아들이는(Handshake) 바로 그 순간, 
    // 외부에서 들어온 편향 값을 낚아채어 안전하게 보관합니다.
    // =========================================================================
    reg [T0*BIAS_WIDTH-1:0] cf_bias;
    always @(posedge clk) begin
        if (!rst_n) begin
            cf_bias <= 0;
        end else if (valid_in && ready_out) begin
            cf_bias <= bias_in; // 픽셀과 생사고락을 함께할 편향 값 확정!
        end
    end

    // =========================================================================
    // 2. Pipeline Balancing Register (1-Clock Delay)
    // =========================================================================
    reg [DATA_WIDTH*FOLD_CH-1:0] delayed_pixel;
    reg delayed_valid;
    reg delayed_last_fold;
    reg [T0*BIAS_WIDTH-1:0] delayed_bias; // 💡 지연된 편향 레지스터 추가

    always @(posedge clk) begin
        if (!rst_n) begin
            delayed_pixel     <= 0;
            delayed_valid     <= 0;
            delayed_last_fold <= 0;
            delayed_bias      <= 0;
        end 
        else if (ready_in) begin
            delayed_pixel     <= cf_folded_pixel;
            delayed_valid     <= cf_valid;
            delayed_last_fold <= cf_last_fold;
            delayed_bias      <= cf_bias; // 💡 픽셀과 100% 동일한 스톨/지연 동기화!
        end
    end

    // =========================================================================
    // 3. 1x1 합성곱 엔진 배열 
    // =========================================================================
    conv_1x1_array #(
        .T0(T0), .FOLD_CH(FOLD_CH), .DATA_WIDTH(DATA_WIDTH), 
        .BIAS_WIDTH(BIAS_WIDTH), .PSUM_WIDTH(PSUM_WIDTH)
    ) u_conv_array (
        .clk          (clk), .rst_n        (rst_n),
        .pixel_in     (delayed_pixel),       
        .weight_in    (weight_in), 
        .bias_in      (delayed_bias), // 💡 직접 연결 척결, 동기화된 편향 주입!
        .valid_in     (delayed_valid),       
        .last_fold_in (delayed_last_fold),   
        .ready_in     (ready_in),      
        .psum_out     (psum_out),   
        .valid_out    (valid_out)
    );

endmodule