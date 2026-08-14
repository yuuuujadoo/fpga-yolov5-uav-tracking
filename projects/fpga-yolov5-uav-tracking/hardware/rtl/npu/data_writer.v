// =========================================================================
// data_writer.v - 출력 수집 + 빽빽패킹 + RMW (모든 conv op 공유)
//   post_processor 의 픽셀 스트림(i_pix/i_valid)을 받아 GFB 에 빽빽패킹 write.
//   다그룹(OUT>16) 출력은 fold-major 패킹으로 한 워드에 여러 그룹 슬롯이 섞이므로,
//   그룹 순차처리 시 grp>0 은 기존 워드를 read-modify-write(RMW) 로 병합.
//
//   [설계 원칙: hold 없는 워드 확정]
//   - 현재 픽셀은 항상 즉시 cur_word 에 기록 (지연/hold 없음)
//   - 워드 확정(write/RMW)은 '이전 워드'를 독립 처리 -> 현재 픽셀과 분리
//   - RMW 2클럭 동안 o_ready=0 으로 backpressure (post_processor stall)
//   - RMW read 는 GFB read 포트를 prefetch 와 시분할 (o_rmw_re 우선)
//
//   [속도] 데이터패스는 스트리밍 유지. RMW 는 워드전환(16슬롯당 1회)에만 2클럭.
//          다그룹이어도 워드당 1회 RMW -> 오버헤드 최소 (실시간 inference 유지).
// =========================================================================
module data_writer #(
    parameter GFB_WIDTH = 2048,
    parameter ADDR_WIDTH = 14,
    parameter T0 = 16,
    parameter DATA_WIDTH = 8
)(
    input  wire clk,
    input  wire rst_n,

    // 제어 (conv_engine 이 그룹 시작 전 설정)
    input  wire        i_start,        // 새 그룹 처리 시작 (1클럭 펄스)
    input  wire [13:0] i_out_base,     // 출력 GFB base 주소
    input  wire [19:0] i_target_pixel, // 출력 픽셀 수 (그룹당)
    input  wire [4:0]  i_out_grps,     // 출력 그룹 수 (OUTG)
    input  wire [4:0]  i_grp,          // 현재 그룹 번호 (0..OUTG-1)

    // 입력 픽셀 스트림 (post_processor 로부터)
    input  wire [T0*DATA_WIDTH-1:0] i_pix,    // 픽셀 (16채널 x 8b = 128b)
    input  wire        i_valid,        // 픽셀 유효
    output wire        o_ready,        // backpressure (0 이면 post 정지)

    // GFB write 포트
    output reg         o_we,
    output reg  [ADDR_WIDTH-1:0] o_waddr,
    output reg  [GFB_WIDTH-1:0]  o_wdata,

    // GFB RMW read 포트 (prefetch 와 시분할; o_rmw_re 우선)
    output reg         o_rmw_re,
    output reg  [ADDR_WIDTH-1:0] o_rmw_raddr,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,

    // 상태
    output reg         o_grp_done,     // 현재 그룹 출력 수집 완료
    output wire        o_flush_done    // 그룹 마지막워드까지 write 완료(완전 idle)
);
    localparam SLOT_W = T0*DATA_WIDTH;  // 128

    // 출력 픽셀 카운터 (그룹 내)
    reg [19:0] opix;
    // 현재 누적 중인 워드
    reg [GFB_WIDTH-1:0] cur_word;
    reg [13:0]  cur_addr;
    reg [15:0]  cur_mask;     // 현재 워드에서 이 그룹이 채운 슬롯
    reg         cur_valid;    // cur_word 에 데이터 있음

    // RMW FSM
    reg [1:0] wstate; localparam W_RUN=0, W_RMWR=1, W_MERGE=2;
    reg [GFB_WIDTH-1:0] rmw_word;   // RMW 대상(이전) 워드 데이터
    reg [13:0]  rmw_addr;
    reg [15:0]  rmw_mask;
    reg [GFB_WIDTH-1:0] merge_word; integer mi;

    wire busy = (wstate != W_RUN);
    assign o_flush_done = o_grp_done && !cur_valid && (wstate==W_RUN);
    // o_ready: registered. RMW 시작과 동시에 0 되어 다음 픽셀(연속스트림)을 막음.
    //   현재 픽셀은 이번 클럭에 이미 수락(조합 W_RUN 처리)되므로 손실 없음.
    assign o_ready = ~busy;  // 통합전 ~rmw_busy 와 동일 타이밍

    // 빽빽패킹 좌표 (조합)
    reg [19:0] gfold; reg [4:0] slot; reg [13:0] waddr_calc;
    always @(*) begin
        gfold = opix * i_out_grps + i_grp;
        slot  = gfold[3:0];         // % 16
        waddr_calc = i_out_base + gfold[19:4];  // / 16
    end

    always @(posedge clk) begin
        if(!rst_n) begin
            opix<=0; cur_word<=0; cur_addr<=0; cur_mask<=0; cur_valid<=0;
            wstate<=W_RUN; o_we<=0; o_rmw_re<=0; o_grp_done<=0; rmw_mask<=0;
        end else begin
            o_we<=0; o_rmw_re<=0;
            if(i_start) begin
                // 새 그룹 시작: 카운터/누적워드 초기화
                opix<=0; cur_word<=0; cur_mask<=0; cur_valid<=0;
                wstate<=W_RUN; o_grp_done<=0;
            end else case(wstate)
                W_RUN: begin
                    if(i_valid && !o_grp_done) begin
                        if(cur_valid && waddr_calc != cur_addr) begin
                            // 워드 전환: 이전 워드(cur_word) 확정
                            if(i_grp==0) begin
                                // grp0: 부분워드 그냥 write
                                o_we<=1; o_waddr<=cur_addr; o_wdata<=cur_word;
                                // 현재 픽셀로 새 워드 시작
                                cur_word <= ({{(GFB_WIDTH-SLOT_W){1'b0}}, i_pix} << (slot*SLOT_W));
                                cur_mask <= (16'd1<<slot); cur_addr<=waddr_calc; cur_valid<=1;
                                opix<=opix+1;
                                if(opix+1>=i_target_pixel) o_grp_done<=1;
                            end else begin
                                // grp>0: 이전 워드 RMW. 현재 픽셀은 RMW 후 새 워드로.
                                //   현재 픽셀을 먼저 cur_word(새워드)에 즉시 기록하고,
                                //   RMW 는 이전워드(rmw_word) 독립 처리 -> hold 불필요.
                                rmw_word<=cur_word; rmw_addr<=cur_addr; rmw_mask<=cur_mask;
                                o_rmw_re<=1; o_rmw_raddr<=cur_addr; wstate<=W_RMWR;
                                // 현재 픽셀로 새 워드 시작 (즉시)
                                cur_word <= ({{(GFB_WIDTH-SLOT_W){1'b0}}, i_pix} << (slot*SLOT_W));
                                cur_mask <= (16'd1<<slot); cur_addr<=waddr_calc; cur_valid<=1;
                                opix<=opix+1;
                                if(opix+1>=i_target_pixel) o_grp_done<=1;
                            end
                        end else begin
                            // 같은 워드: 슬롯 채우기
                            cur_word[slot*SLOT_W +: SLOT_W] <= i_pix;
                            cur_mask <= cur_mask | (16'd1<<slot);
                            cur_addr <= waddr_calc; cur_valid<=1;
                            opix<=opix+1;
                            if(opix+1>=i_target_pixel) o_grp_done<=1;
                        end
                    end
                end
                W_RMWR: begin
                    // RMW read 요청한 워드가 1클럭 뒤 i_gfb_rdata 에 도착
                    wstate<=W_MERGE;
                end
                W_MERGE: begin
                    merge_word = i_gfb_rdata;
                    for(mi=0;mi<16;mi=mi+1)
                        if(rmw_mask[mi]) merge_word[mi*SLOT_W +: SLOT_W] = rmw_word[mi*SLOT_W +: SLOT_W];
                    o_we<=1; o_waddr<=rmw_addr; o_wdata<=merge_word;
                    wstate<=W_RUN;
                end
            endcase

            // 그룹 끝: 마지막 워드(cur_word) 확정 (o_grp_done 직후 1회)
            // o_grp_done set 된 다음 클럭에 flush
            if(o_grp_done && cur_valid && wstate==W_RUN) begin
                if(i_grp==0) begin
                    o_we<=1; o_waddr<=cur_addr; o_wdata<=cur_word; cur_valid<=0;
                end else begin
                    rmw_word<=cur_word; rmw_addr<=cur_addr; rmw_mask<=cur_mask;
                    o_rmw_re<=1; o_rmw_raddr<=cur_addr; wstate<=W_RMWR; cur_valid<=0;
                end
            end
        end
    end
endmodule
