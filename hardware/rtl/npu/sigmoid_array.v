`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 모듈명: sigmoid_array (6-Channel Parallel Activation)
// 역할: 
// - Splitter에서 찢어준 6개의 데이터(tx, ty, tw, th, conf, cls)를 동시에 Sigmoid에 통과시킵니다.
// - Sigmoid 메모리 읽기에 1클럭(Latency)이 소모되므로, 연산하지 않고 지나가는 
// 좌표값(cx, cy, anchor)들도 1클럭 지연시켜 최종 출력 타이밍을 완벽하게 맞춰줍니다.
//////////////////////////////////////////////////////////////////////////////////

module sigmoid_array #(
    parameter DATA_WIDTH = 8,       // 1개 변수 데이터 폭
    parameter MAX_COORD_BITS = 8    // 좌표 레지스터 비트 폭
)(
    // [시스템 공통 포트]
    input  wire                   clk,        // 메인 클럭
    input  wire                   rst_n,      // 비동기 리셋

    // [입력 포트] Data Splitter로부터 들어오는 6개 변수와 좌표 메타데이터
    input  wire                   i_valid,    // 입력 데이터 유효성
    input  wire                   i_ready,    // 하위 모듈이 보내는 파이프라인 정지 신호 (브로드캐스팅 용도)
    
    input  wire [DATA_WIDTH-1:0]  i_tx,       // X 변위량
    input  wire [DATA_WIDTH-1:0]  i_ty,       // Y 변위량
    input  wire [DATA_WIDTH-1:0]  i_tw,       // 너비 변위량
    input  wire [DATA_WIDTH-1:0]  i_th,       // 높이 변위량
    input  wire [DATA_WIDTH-1:0]  i_tconf,    // 객체 신뢰도
    input  wire [DATA_WIDTH-1:0]  i_tcls,     // 클래스 확률
    
    input  wire [MAX_COORD_BITS-1:0] i_cx,    // Bypass 시킬 X 좌표
    input  wire [MAX_COORD_BITS-1:0] i_cy,    // Bypass 시킬 Y 좌표
    input  wire [1:0]                i_anchor,// Bypass 시킬 앵커 인덱스

    // [출력 포트] Sigmoid 연산을 마친 6개 결과 및 타이밍이 맞춰진 좌표 데이터
    output wire                   o_valid,    // 결과 데이터 유효성
    
    output wire [DATA_WIDTH-1:0]  o_tx,
    output wire [DATA_WIDTH-1:0]  o_ty,
    output wire [DATA_WIDTH-1:0]  o_tw,
    output wire [DATA_WIDTH-1:0]  o_th,
    output wire [DATA_WIDTH-1:0]  o_tconf,
    output wire [DATA_WIDTH-1:0]  o_tcls,
    
    // Bypass 레지스터 출력 선언 (연산 결과를 기다리며 1클럭을 쉬고 나온 값들)
    output reg  [MAX_COORD_BITS-1:0] o_cx,    
    output reg  [MAX_COORD_BITS-1:0] o_cy,    
    output reg  [1:0]                o_anchor 
);

    // =========================================================================
    // [LUT Core 인스턴스화 구역]
    // 설명: 6개의 변수가 병목 없이 1클럭 만에 동시에 처리되도록 메모리를 6개 복제하여 병렬 배치합니다.
    // =========================================================================
    
    // 6개 모듈에서 뿜어져 나오는 valid 신호를 담아둘 내부 전선 배열
    wire [5:0] w_valid_out;
    
    // 6개의 LUT는 동일한 클럭 지연(1-Clock)을 가지므로, 대표로 0번 놈의 신호만 바깥으로 뺍니다.
    assign o_valid = w_valid_out[0];

    // 1번: tx 전용 Sigmoid LUT
    sigmoid_lut u_lut_tx (
        .clk(clk), .rst_n(rst_n), 
        .din(i_tx), .valid_in(i_valid), .ready_in(i_ready), // i_ready를 뿌려주어 일제히 멈출 수 있게 함
        .dout(o_tx), .valid_out(w_valid_out[0])
    );

    // 2번: ty 전용 Sigmoid LUT
    sigmoid_lut u_lut_ty (
        .clk(clk), .rst_n(rst_n), 
        .din(i_ty), .valid_in(i_valid), .ready_in(i_ready),
        .dout(o_ty), .valid_out(w_valid_out[1])
    );

    // 3번: tw 전용 Sigmoid LUT
    sigmoid_lut u_lut_tw (
        .clk(clk), .rst_n(rst_n), 
        .din(i_tw), .valid_in(i_valid), .ready_in(i_ready),
        .dout(o_tw), .valid_out(w_valid_out[2])
    );

    // 4번: th 전용 Sigmoid LUT
    sigmoid_lut u_lut_th (
        .clk(clk), .rst_n(rst_n), 
        .din(i_th), .valid_in(i_valid), .ready_in(i_ready),
        .dout(o_th), .valid_out(w_valid_out[3])
    );

    // 5번: conf 전용 Sigmoid LUT
    sigmoid_lut u_lut_conf (
        .clk(clk), .rst_n(rst_n), 
        .din(i_tconf), .valid_in(i_valid), .ready_in(i_ready),
        .dout(o_tconf), .valid_out(w_valid_out[4])
    );

    // 6번: cls 전용 Sigmoid LUT
    sigmoid_lut u_lut_cls (
        .clk(clk), .rst_n(rst_n), 
        .din(i_tcls), .valid_in(i_valid), .ready_in(i_ready),
        .dout(o_tcls), .valid_out(w_valid_out[5])
    );

    // =========================================================================
    // [always 블록] 바이패스 신호 타이밍 동기화 (1-Clock Delay)
    // 설명: 위에서 Sigmoid LUT를 통과하는 데 1클럭이 걸립니다. 
    // 연산하지 않는 좌표들도 레지스터 플립플롭을 한 번 거치게 만들어 똑같이 1클럭을 쉬게 만듭니다.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 리셋 시 좌표값 청소
            o_cx <= 0; o_cy <= 0; o_anchor <= 0;
        end 
        else begin
            // 멈춤 신호(i_ready)가 1일 때만 동기화 파이프라인이 흐릅니다. (Absolute Freeze 방어)
            if (i_ready) begin
                
                // 데이터가 유효할 때만 이전 좌표를 현재 좌표 레지스터에 덮어씁니다.
                if (i_valid) begin
                    o_cx     <= i_cx;
                    o_cy     <= i_cy;
                    o_anchor <= i_anchor;
                end
                
            end
        end
    end

endmodule