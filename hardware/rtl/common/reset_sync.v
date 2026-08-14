`timescale 1ns/1ps
//============================================================================
// reset_sync.v
//----------------------------------------------------------------------------
//  비동기 리셋의 "동기 해제(synchronous deassertion)" 동기화기.
//
//  PCS/PMA core_resets(200MHz)에서 나온 리셋은 이더넷 125MHz(clkout0/userclk2)
//  레지스터의 R/PRE 핀에 직접 닿아 CDC-1/CDC-7 Critical을 유발한다.
//
//  본 모듈을 125MHz 도메인 클럭으로 인스턴스하여 i_rst_async 를 통과시키면:
//   - assert(인가)는 즉시 (비동기)
//   - deassert(해제)는 클럭에 동기, 4FF 체인으로 복구 분포 보장
//  -> 모든 다운스트림 R/PRE는 동일 클럭에서 동시에 풀리므로 Critical 제거.
//
//  타이밍/SI: ASYNC_REG로 4 FF가 같은 슬라이스에 패킹 -> 메타스테이블 MTBF 극대화.
//============================================================================
module reset_sync (
    input  wire i_clk,         // 목적지 도메인 클럭 (예: gtx 125MHz)
    input  wire i_rst_async,   // 비동기 리셋 입력 (active-high)
    output wire o_rst_sync     // 동기 해제 리셋 (active-high)
);

    (* ASYNC_REG = "TRUE" *) reg [3:0] r_sync;

    always @(posedge i_clk or posedge i_rst_async) begin
        if (i_rst_async) r_sync <= 4'b1111;
        else             r_sync <= {r_sync[2:0], 1'b0};
    end

    assign o_rst_sync = r_sync[3];

endmodule
