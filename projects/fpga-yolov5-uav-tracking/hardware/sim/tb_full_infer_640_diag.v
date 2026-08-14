`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_full_infer_640_diag.v  -  진단용: NPU 가 96-bit bbox 를 실제로 내보내는지 확인
//   목적: full_infer 의 "캡처=0" 이 (A) TB 캡처 문제인지 (B) NPU 미출력인지 판별.
//   방법: 추론 전 구간(+여유) 동안 m_axis_tvalid 가 1 이 되는 모든 순간을 기록.
//         - tvalid 가 한 번이라도 1  -> (A) NPU 는 출력함. TB 캡처 윈도우만 문제.
//         - tvalid 가 끝까지 0       -> (B) NPU 가 bbox 를 안 내보냄. 실제 문제.
//   (값 비교는 하지 않는다. 오직 "출력이 나오는가/몇 개인가/언제부터인가" 만 본다.)
//////////////////////////////////////////////////////////////////////////////////

module tb_full_infer_640_diag;

    parameter IMG_WORDS = 5120;
    parameter IMG_BASE   = 4480;
    parameter NUM_STEPS  = 57;   // 640 그래프 = 57 스텝 (헤드 detect = lid 56). 55면 헤드 누락→bbox 0개

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg          i_load_en;
    reg  [13:0]  ld_waddr;
    reg          ld_we;
    reg  [2047:0] ld_wdata;
    reg          i_start;
    wire         o_all_done;
    wire [95:0]  axi_data;
    wire         axi_valid;
    reg          axi_ready;

    // NMS_THRESH(0)=pass-all -> 골든(sl_full_axi_gold640.mem, NMS 이전 24000개)과 동일 조건.
    //   실제 보드는 npu_top 기본값 13 사용(검출만 통과). 검증만 0 으로 오버라이드.
    npu_top #(.NUM_STEPS(NUM_STEPS), .NMS_THRESH(8'd0)) dut (
        .clk(clk), .rst_n(rst_n),
        .i_load_en(i_load_en), .i_ld_waddr(ld_waddr), .i_ld_we(ld_we), .i_ld_wdata(ld_wdata),
        .i_start(i_start), .o_all_done(o_all_done),
        .m_axis_tdata(axi_data), .m_axis_tvalid(axi_valid), .m_axis_tready(axi_ready)
    );

    reg [2047:0] img [0:8191];
    integer i;
    integer valid_count;        // tvalid&ready 가 1 이었던 클럭 수 (= 받은 박스 수)
    integer first_time;         // 첫 tvalid 시각(ns). -1 이면 한 번도 없었음
    reg     all_done_seen;
    integer done_time;

    initial begin
        i_load_en=0; ld_we=0; i_start=0; axi_ready=1;
        valid_count=0; first_time=-1; all_done_seen=0; done_time=-1;

        $readmemh("lp_full640.mem",     dut.u_core.u_prom.param_rom);
        $readmemh("sl_full640_img.mem", img);

        #20 rst_n=1; @(posedge clk);

        i_load_en<=1; @(posedge clk);
        for (i=0;i<IMG_WORDS;i=i+1) begin
            ld_waddr<=IMG_BASE+i; ld_wdata<=img[i]; ld_we<=1; @(posedge clk);
        end
        ld_we<=0; i_load_en<=0; @(posedge clk);

        i_start<=1; @(posedge clk); i_start<=0;
        $display("==== [DIAG] 640 추론 시작: m_axis 출력 감시 ====");
    end

    // ----------------------------------------------------------------------
    // [감시] 매 클럭 tvalid 를 본다. all_done 이후에도 충분히(20만 클럭) 더 본다.
    // ----------------------------------------------------------------------
    integer extra;
    initial begin
        @(posedge rst_n);
        extra = 0;
        forever begin
            @(posedge clk);

            // m_axis 출력 순간 기록
            if (axi_valid) begin
                if (first_time < 0) begin
                    first_time = $time;
                    $display("  [DIAG] ★ 첫 m_axis_tvalid=1 @ %0t ns  (NPU 가 출력 중!)", $time);
                end
                if (axi_ready) valid_count = valid_count + 1;
                // 처음 5개 박스는 값도 보여줌
                if (valid_count <= 5)
                    $display("    box#%0d tdata=%024X (lid=%0d)", valid_count, axi_data, dut.u_core.layer_id);
            end

            // all_done 시각 기록
            if (o_all_done && !all_done_seen) begin
                all_done_seen = 1;
                done_time = $time;
                $display("  [DIAG] o_all_done=1 @ %0t ns (지금까지 받은 박스 %0d 개)", $time, valid_count);
            end

            // all_done 후 200000 클럭 더 보고 종료
            if (all_done_seen) begin
                extra = extra + 1;
                if (extra > 200000) begin
                    $display("==================================================");
                    $display("[DIAG 결과] 총 받은 박스 = %0d 개", valid_count);
                    if (valid_count > 0)
                        $display(">>> (A) NPU 는 bbox 를 정상 출력함. 첫 출력 @ %0t ns. full_infer 캡처는 TB 문제였음 <<<", first_time);
                    else
                        $display(">>> (B) NPU 가 bbox 를 전혀 안 내보냄 (tvalid 끝까지 0). 헤드 출력 경로 점검 필요 <<<");
                    $finish;
                end
            end
        end
    end

    // 안전 타임아웃
    initial begin
        #(3_000_000 * 1000);
        $display("[DIAG TIMEOUT] valid_count=%0d first_time=%0t all_done=%0d", valid_count, first_time, all_done_seen);
        $finish;
    end

endmodule
