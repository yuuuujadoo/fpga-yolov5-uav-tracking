`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: requant_array
// Description:
//   - 단일 채널 requantization 모듈을 T0(16)개 병렬로 인스턴스화(복제)하는 래퍼(Wrapper) 모듈
//   - 16개의 Conv 엔진에서 나온 32-bit Psum을 동시에 입력받아 각각 16-bit로 압축
//   - 레이어의 M0와 n 설정값은 16개 코어에 동일하게 브로드캐스팅(Broadcasting) 됨
//   - [추가] 외부 ready_in 신호를 16개 코어에 브로드캐스팅하여 동시 정지 제어
//////////////////////////////////////////////////////////////////////////////////

module requant_array #(
    parameter T0 = 16,               // 출력 채널 병렬도
    parameter PSUM_WIDTH = 32,       // 개별 Psum 비트 수
    parameter M0_WIDTH = 16,         // 개별 M0 비트 수
    parameter OUT_WIDTH = 8          // 개별 출력 비트 수
)(
    input wire clk, rst_n,

    // [입력 데이터 버스] 16채널 묶음
    input wire [T0*M0_WIDTH-1:0]   M0_in,     // 16-bit * 16채널
    input wire [T0*5-1:0]          n_in,      // 5-bit * 16채널
    input wire [T0*PSUM_WIDTH-1:0] psum_in,   // 32-bit * 16채널
    
    input wire valid_in,                      // 데이터 묶음 유효 깃발
    input wire ready_in,                      // 💡 [추가] 뒷단이 멈추라고 보내는 신호

    // [출력 데이터 버스] 16채널 묶음
    output wire ready_out,                    // 💡 [추가] 앞단으로 보내는 멈춤 신호
    output wire [T0*OUT_WIDTH-1:0] requant_out,// 8-bit * 16채널 결과 묶음
    output wire valid_out                     // 결과 묶음 유효 깃발
);

    // 하위 모듈들의 출력을 담을 1차원 배열(Wire Array) 선언
    wire [OUT_WIDTH-1:0] w_requant_out [0:T0-1];
    wire [T0-1:0] w_valid_out;
    wire [T0-1:0] w_ready_out;

    // 16개 모듈은 모두 똑같이 동작하므로, 0번 채널의 제어 신호만 대표로 출력에 묶습니다.
    assign valid_out = w_valid_out[0];
    assign ready_out = w_ready_out[0];

    genvar i;
    generate
        for (i = 0; i < T0; i = i + 1) begin : REQUANT_CORES
            // [Unpacking] 입력으로 들어온 거대 버스에서 자신의 채널(i) 구간만 잘라냅니다.
            wire [PSUM_WIDTH-1:0] current_psum = psum_in[(i * PSUM_WIDTH) +: PSUM_WIDTH];
            wire [M0_WIDTH-1:0]   current_M0   = M0_in[(i * M0_WIDTH) +: M0_WIDTH];
            wire [4:0]            current_n    = n_in[(i * 5) +: 5];

            requantization #(
                .PSUM_WIDTH(PSUM_WIDTH), .M0_WIDTH(M0_WIDTH), .OUT_WIDTH(OUT_WIDTH)
            ) u_req (
                .clk(clk), .rst_n(rst_n),
                .M0_in(current_M0), .n_in(current_n), .psum_in(current_psum),
                .valid_in(valid_in),
                .ready_in(ready_in), // 💡 16개 코어에 멈춤 신호 브로드캐스팅
                .ready_out(w_ready_out[i]),
                .requant_out(w_requant_out[i]), // 개별 출력 결과 배열에 저장
                .valid_out(w_valid_out[i])
            );
        end
    endgenerate

    // [Packing] 하위 모듈에서 나온 개별 출력들을 다시 거대 버스 하나로 이어 붙입니다.
    generate
        for (i = 0; i < T0; i = i + 1) begin : PACK_OUT
            assign requant_out[(i * OUT_WIDTH) +: OUT_WIDTH] = w_requant_out[i];
        end
    endgenerate

endmodule