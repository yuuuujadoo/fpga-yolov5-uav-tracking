`timescale 1ns / 1ps
// =================================================================================
// 모듈명: nms_prefilter  (1차 NMS = confidence prefilter)
// 역할:
//   - head(yolo_head_wrapper) 가 내보내는 96b BBox 스트림을 인라인 통과시키며,
//     신뢰도(conf, 또는 conf*cls)가 임계값 미만인 박스를 '걸러' 외부로 보내지 않음.
//   - IoU 억제(진짜 NMS)는 하지 않음 -> 그건 PC(2차). 여기선 1차 신뢰도 게이트만.
//   - 통과 박스에는 원래 셀 인덱스(raster, P3->P4)를 부착해 PC 가 위치 복원 가능.
//
// 프로젝트 방향성 반영:
//   - 자원 최소: 비교기 1 + 카운터 몇 개. BRAM 0. USE_CLS=0 이면 DSP 0(conf 게이트만),
//     USE_CLS=1 이면 8x8 곱셈 1개(conf*cls). 기본 0.
//   - 실시간(delay): 1 박스/클럭 스트리밍, 조합 비교 + 1단 레지스터. 기존 head 출력
//     스트림을 '통과시키며 거르기'만 하므로 inference 사이클(6.26M)에 가산 없음.
//   - backpressure 보존: 외부 FIFO stall(o_tready=0) -> 상류 head 까지 정지(i_tready).
//
// 인덱스 부착(폭 불변):
//   - 96b 의 conf/cls 필드 상위 8b 는 zero-padding({8'h00,...}) 임.
//   - 그 패딩 바이트에 16b cell_index 를 실어보냄 -> 96b 폭 유지, 실데이터 불변.
//     o_tdata[95:88] = index[15:8],  o_tdata[79:72] = index[7:0]
//   - PC 는 index 로 (head=P3/P4, cy, cx, anchor) 복원: idx<19200 -> P3, 그 외 P4.
//
// 임계값: i_thresh (Q0.7, conf 와 동일 스케일). 0 이면 전부 통과(=프리필터 무효, 회귀용).
// =================================================================================
module nms_prefilter #(
    parameter IDX_WIDTH = 16,   // cell index 비트폭 (24000 < 2^16)
    parameter USE_CLS   = 0     // 0: conf 게이트(DSP 0), 1: (conf*cls)>>7 게이트(곱셈 1)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_frame_start,   // 프레임(head) 시작 펄스 -> index/카운터 리셋
    input  wire [7:0]  i_thresh,        // Q0.7 임계값 (0 = pass-all)

    // ===== head(yolo_head_wrapper) 96b 스트림 직결 =====
    input  wire [95:0] i_tdata,
    input  wire        i_tvalid,
    output wire        i_tready,         // -> head backpressure

    // ===== 외부 FIFO/이더넷 (통과 박스만) =====
    output reg  [95:0] o_tdata,
    output reg         o_tvalid,
    input  wire        o_tready,

    // ===== 통계(선택) =====
    output reg  [IDX_WIDTH-1:0] o_pass_count,    // 통과 박스 수
    output reg  [IDX_WIDTH-1:0] o_total_count    // 총 입력 박스 수
);
    // ----- 신뢰도 추출 (96b 배치: cls[87:80], conf[71:64]) -----
    wire [7:0] conf = i_tdata[64 +: 8];
    wire [7:0] cls  = i_tdata[80 +: 8];

    // ----- score (Q0.7) -----
    //   USE_CLS=0: conf 단독 (nc=1 이라 objectness 단독이 강력한 1차 게이트, DSP 0)
    //   USE_CLS=1: (conf*cls)>>7  (YOLO 최종 score 근사, 곱셈 1개)
    wire [15:0] cs_q14 = conf * cls;            // Q0.14
    wire [7:0]  score  = (USE_CLS != 0) ? cs_q14[14:7] : conf;
    wire        pass   = (score >= i_thresh);

    // ----- 인덱스 카운터 (raster, P3->P4) -----
    reg [IDX_WIDTH-1:0] idx;

    // ----- backpressure: 출력이 막히면 상류도 정지 -----
    wire stall = o_tvalid & ~o_tready;
    assign i_tready = ~stall;

    // 통과 박스 96b: 패딩 바이트에 index 삽입 (실데이터 보존)
    wire [95:0] tag_box = { idx[15:8], i_tdata[87:80],     // [95:88]=idx_hi, [87:80]=cls
                           idx[7:0],  i_tdata[71:64],      // [79:72]=idx_lo, [71:64]=conf
                           i_tdata[63:0] };                // bh,bw,by,bx 그대로

    always @(posedge clk) begin
        if(!rst_n) begin
            o_tvalid<=1'b0; o_tdata<=96'd0; idx<={IDX_WIDTH{1'b0}};
            o_pass_count<={IDX_WIDTH{1'b0}}; o_total_count<={IDX_WIDTH{1'b0}};
        end else begin
            if(i_frame_start) begin
                idx<={IDX_WIDTH{1'b0}};
                o_pass_count<={IDX_WIDTH{1'b0}}; o_total_count<={IDX_WIDTH{1'b0}};
            end
            if(!stall) begin
                o_tvalid <= 1'b0;                 // 기본: 출력 없음(걸러짐)
                if(i_tvalid) begin
                    o_total_count <= o_total_count + 1'b1;
                    if(pass) begin
                        o_tdata     <= tag_box;
                        o_tvalid    <= 1'b1;
                        o_pass_count<= o_pass_count + 1'b1;
                    end
                    idx <= idx + 1'b1;            // 입력 박스마다 셀 인덱스 증가
                end
            end
            // stall 중에는 o_tvalid/o_tdata 유지(hold), 상류는 i_tready=0 로 정지
        end
    end
endmodule
