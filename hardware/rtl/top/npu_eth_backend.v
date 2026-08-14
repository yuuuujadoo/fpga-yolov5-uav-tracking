`timescale 1ns / 1ps
//============================================================================
//  npu_eth_backend.v
//----------------------------------------------------------------------------
//  역할
//   - 검증된 32-bit "BRAM 스타일" 명령 포트(=oss_udp_bram_endpoint 의 bram_*
//     포트, 또는 UART 프레임 엔드포인트의 동일 포트) 뒤에 NPU 를 직접
//     메모리맵으로 붙인다.  DDR3/MIG/MicroBlaze/AXI-SmartConnect 를 전부
//     제거하고, 이미지를 UDP/UART -> GFB 적재포트로 곧바로 스트리밍한다.
//
//  명령 포트(슬레이브)  : oss_udp_bram_endpoint 와 1:1 동일 타이밍
//   - i_cmd_en       : 1클럭 액세스 스트로브 (read/write 공통)
//   - i_cmd_we[3:0]  : 4'hf = 32-bit write, 4'h0 = read
//   - i_cmd_addr[31:0]: 바이트 주소 (워드=4바이트 정렬)
//   - i_cmd_wdata    : write 데이터
//   - o_cmd_rdata    : read 데이터 (액세스 1클럭 뒤 유효 = BRAM 등가)
//      * 엔드포인트 read 시퀀스(REQ->WAIT->CAPTURE)와 정확히 맞물린다.
//
//  주소맵 (바이트 주소 상위 니블 i_cmd_addr[31:28] 로 영역 디코드)
//   - 0x0_______ : IMG    (write) 이미지 워드.  32-bit 64개 -> 2048-bit 1워드
//                  를 조립해 GFB 적재포트(o_ld_*)로 흘려보낸다.
//                  GFB 워드주소 = addr>>8,  레인(0..63) = addr[7:2]
//   - 0x1_______ : CTRL   (write) bit0=START (추론 시작 펄스)
//   - 0x2_______ : STATUS (read)  {box_count[15:0], 14'd0, done_sticky, busy}
//   - 0x3_______ : RESULT (read)  word0 = box_count
//                  word(1+3k+0/1/2) = box k 의 [31:0]/[63:32]/[95:64]
//
//  NPU 측(마스터)  : npu_top 의 포트와 1:1
//   - o_load_en/o_ld_waddr/o_ld_we/o_ld_wdata : 이미지 적재
//   - o_start / i_all_done                    : 추론 제어
//   - i_bbox_tdata[95:0]/i_bbox_tvalid/o_bbox_tready : BBox 수집
//
//  설계 메모
//   - 전부 단일 클럭(app_clk) 동기.  CDC 없음(이더넷 도메인 경계는 상위의
//     axis_async_fifo 가 담당).
//   - 모든 메모리(box_mem)는 작은 분산RAM 수준(레지스터 어레이).  GFB 조립
//     레지스터(asm) 1개(2048-bit)만 큰 폭이며 이는 FF 로 가도 무방한 크기.
//   - Verilog-2001 문법만 사용(SystemVerilog 아님).
//============================================================================
module npu_eth_backend #(
    parameter integer GFB_WIDTH  = 2048,  // GFB 워드 폭 (= npu_top.GFB_WIDTH)
    parameter integer ADDR_WIDTH = 14,    // GFB 주소 폭 (= npu_top.ADDR_WIDTH)
    parameter integer MAX_BOXES  = 256    // 수집 가능한 최대 BBox 수
)(
    input  wire                   clk,
    input  wire                   rst,        // active-high (app_rst 와 동일)

    //---------------- 명령 포트 (32-bit BRAM 슬레이브) ----------------
    input  wire        [31:0]     i_cmd_addr,
    input  wire        [31:0]     i_cmd_wdata,
    output reg         [31:0]     o_cmd_rdata,
    input  wire                   i_cmd_en,
    input  wire        [3:0]      i_cmd_we,

    //---------------- NPU 적재/제어/결과 (npu_top 마스터) ----------------
    output wire                   o_load_en,
    output reg  [ADDR_WIDTH-1:0]  o_ld_waddr,
    output reg                    o_ld_we,
    output reg  [GFB_WIDTH-1:0]   o_ld_wdata,

    output reg                    o_start,
    input  wire                   i_all_done,

    input  wire        [95:0]     i_bbox_tdata,
    input  wire                   i_bbox_tvalid,
    output wire                   o_bbox_tready,

    //---------------- 디버그/상태 (상위 LED/카운터용, 선택) ----------------
    output wire                   o_busy,
    output wire                   o_done,
    output wire        [15:0]     o_box_count
);
    //------------------------------------------------------------------
    // 파생 상수
    //------------------------------------------------------------------
    localparam integer LANES      = GFB_WIDTH/32;     // 2048/32 = 64
    localparam integer LANE_BITS  = 6;                // log2(64)
    // 영역 디코드(상위 니블)
    localparam [3:0] REGION_IMG    = 4'h0;
    localparam [3:0] REGION_CTRL   = 4'h1;
    localparam [3:0] REGION_STATUS = 4'h2;
    localparam [3:0] REGION_RESULT = 4'h3;

    //------------------------------------------------------------------
    // 실행 상태 (run / done)
    //   - run_active : start 펄스 ~ all_done 사이 1
    //   - done_sticky: all_done 이후 1 유지(다음 start 에서 클리어)
    //   - o_load_en  : 추론중이 아닐 때(=적재 가능) 1
    //                  -> 첫 적재/매 프레임 적재 모두 자동으로 load 모드
    //------------------------------------------------------------------
    reg run_active;
    reg done_sticky;
    assign o_load_en   = ~run_active;
    assign o_busy      = run_active;
    assign o_done      = done_sticky;

    //------------------------------------------------------------------
    // 명령 디코드(조합)
    //------------------------------------------------------------------
    wire        is_write   = i_cmd_en & (i_cmd_we != 4'd0);
    wire        is_read    = i_cmd_en & (i_cmd_we == 4'd0);
    wire [3:0]  region     = i_cmd_addr[31:28];

    wire        wr_img     = is_write & (region == REGION_IMG);
    wire        wr_ctrl    = is_write & (region == REGION_CTRL);

    // IMG : GFB 워드주소 / 레인
    wire [ADDR_WIDTH-1:0] img_word_addr = i_cmd_addr[ADDR_WIDTH+8-1 : 8]; // addr>>8
    wire [LANE_BITS-1:0]  img_lane      = i_cmd_addr[7:2];                // (addr>>2)%64
    wire                  img_lane_last = (img_lane == (LANES-1));

    //------------------------------------------------------------------
    // GFB 워드 조립 레지스터
    //   - 패킷 경계와 무관하게 레인 0..63 순서로 채워진다(엔드포인트는
    //     절대주소 기반 순차 쓰기이므로 보장됨).
    //   - 레인 63 쓰기 순간에 완성된 2048-bit 를 o_ld_* 로 1클럭 방출.
    //------------------------------------------------------------------
    reg [GFB_WIDTH-1:0] asm;

    always @(posedge clk) begin
        if (rst) begin
            asm        <= {GFB_WIDTH{1'b0}};
            o_ld_we    <= 1'b0;
            o_ld_waddr <= {ADDR_WIDTH{1'b0}};
            o_ld_wdata <= {GFB_WIDTH{1'b0}};
        end else begin
            o_ld_we <= 1'b0;                       // 기본값(1클럭 펄스)
            if (wr_img) begin
                if (!img_lane_last) begin
                    // 하위 레인 적재
                    asm[img_lane*32 +: 32] <= i_cmd_wdata;
                end else begin
                    // 마지막(63) 레인 도착 -> 이번 데이터를 최상위에 끼워 완성
                    // 완성 워드 = { 이번_레인63, 기존_asm[2015:0] }
                    o_ld_wdata <= {i_cmd_wdata, asm[GFB_WIDTH-33 : 0]};
                    o_ld_waddr <= img_word_addr;
                    o_ld_we    <= 1'b1;
                end
            end
        end
    end

    //------------------------------------------------------------------
    // 제어(START) / 실행상태 갱신
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            o_start     <= 1'b0;
            run_active  <= 1'b0;
            done_sticky <= 1'b0;
        end else begin
            o_start <= 1'b0;                       // 기본값(1클럭 펄스)
            // CTRL.bit0 = START
            if (wr_ctrl && i_cmd_wdata[0]) begin
                o_start     <= 1'b1;
                run_active  <= 1'b1;
                done_sticky <= 1'b0;
            end
            // 추론 완료
            if (i_all_done && run_active) begin
                run_active  <= 1'b0;
                done_sticky <= 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // BBox 수집
    //   - 추론중(run_active)일 때만 수신, 96-bit 박스를 box_mem 에 적재.
    //   - 공간이 남아있을 때만 ready (오버플로 방지).
    //------------------------------------------------------------------
    reg [95:0] box_mem [0:MAX_BOXES-1];
    reg [15:0] box_count;
    assign o_box_count   = box_count;
    assign o_bbox_tready = run_active & (box_count < MAX_BOXES);

    always @(posedge clk) begin
        if (rst) begin
            box_count <= 16'd0;
        end else begin
            // 새 추론 시작 시 카운터 리셋(START 와 동일 조건)
            if (wr_ctrl && i_cmd_wdata[0]) begin
                box_count <= 16'd0;
            end else if (i_bbox_tvalid && o_bbox_tready) begin
                box_mem[box_count[15:0]] <= i_bbox_tdata;
                box_count <= box_count + 16'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // READ 경로 (1클럭 레이턴시, BRAM 등가)
    //   - 엔드포인트 read 시퀀스: en/addr 유효(ST_RD_WAIT) -> 1클럭 뒤
    //     bram_dout 샘플(ST_RD_CAPTURE).  여기 레지스터 출력이 정확히 맞음.
    //   - RESULT word0 = box_count, 이후 3워드/박스.
    //------------------------------------------------------------------
    // 영역 내부(=상위 니블 제외) 워드 인덱스
    wire [25:0] result_widx = i_cmd_addr[27:2];
    // box 인덱스 / 서브워드(0,1,2)  (word0 은 count 이므로 -1)
    wire [25:0] res_box_idx = (result_widx - 26'd1) / 26'd3;
    wire [1:0]  res_sub     = (result_widx - 26'd1) % 26'd3;

    // 박스 메모리 비동기 읽기 -> 다음 단계에서 레지스터링
    reg  [95:0] res_box;
    always @(*) res_box = box_mem[res_box_idx[7:0]];

    always @(posedge clk) begin
        if (rst) begin
            o_cmd_rdata <= 32'd0;
        end else if (is_read) begin
            case (region)
                REGION_STATUS:
                    o_cmd_rdata <= {box_count, 14'd0, done_sticky, run_active};
                REGION_RESULT:
                    if (result_widx == 26'd0)
                        o_cmd_rdata <= {16'd0, box_count};
                    else case (res_sub)
                        2'd0:    o_cmd_rdata <= res_box[31:0];
                        2'd1:    o_cmd_rdata <= res_box[63:32];
                        default: o_cmd_rdata <= res_box[95:64];
                    endcase
                default: o_cmd_rdata <= 32'd0; // IMG/CTRL read = 0
            endcase
        end
    end

endmodule
