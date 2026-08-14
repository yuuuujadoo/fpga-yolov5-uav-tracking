`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_full_infer_640.v  -  640x640 전체 추론 -> 최종 96-bit BBox 출력을 골든과 대조
//   목적: 체크포인트 TB(tb_cp_all_640) 가 통과한 뒤, "최종 출력까지" 동일한지 확인.
//   원리: 입력 적재 -> 추론 -> m_axis 로 나오는 96-bit 레코드를
//         sl_full_axi_gold640.mem 와 순서대로 비교.
//   인덱스 태그 바이트는 마스킹하고 비교(원본 tb_full_infer 와 동일 규칙).
//
//   필요 파일: lp_full640.mem, sl_full640_img.mem, sl_full_axi_gold640.mem
//////////////////////////////////////////////////////////////////////////////////

module tb_full_infer_640;

    parameter IMG_WORDS = 5120;     // 640행 x 8워드
    parameter IMG_BASE   = 4480;    // GFB_DEPTH 9600 - 5120
    parameter NUM_STEPS  = 57;   // 640 그래프 = 57 스텝 (헤드 detect = lid 56). 55면 헤드 누락→bbox 0개
    parameter N_AXI      = 24000;   // 골든 박스 레코드 수 (sl_full_axi_gold640.mem 줄 수)

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

    reg [2047:0] img  [0:8191];
    reg [95:0]   gold [0:24575];      // N_AXI 이상 여유
    integer i, cap, errs;

    // 인덱스 태그 바이트([95:88],[79:72]) 마스킹 - 값 비교에서 셀 인덱스 차이는 무시
    //   (원본 tb_full_infer 의 마스크와 동일)
    localparam [95:0] MASK = 96'h00FF00FFFFFFFFFFFFFFFFFF;

    // 캡처된 한 박스를 골든과 비교
    task check_one;
        begin
            if (axi_valid && axi_ready) begin
                if (cap < N_AXI) begin
                    if ((axi_data & MASK) !== (gold[cap] & MASK)) begin
                        if (errs < 12)
                            $display("[FAIL] axi#%0d (lid=%0d) exp=%024X act=%024X",
                                     cap, dut.u_core.layer_id, gold[cap], axi_data);
                        errs = errs + 1;
                    end
                end
                cap = cap + 1;
            end
        end
    endtask

    initial begin
        i_load_en = 0; ld_we = 0; i_start = 0; axi_ready = 1; cap = 0; errs = 0;

        $readmemh("lp_full640.mem",        dut.u_core.u_prom.param_rom);
        $readmemh("sl_full640_img.mem",    img);
        $readmemh("sl_full_axi_gold640.mem", gold);

        #20 rst_n = 1; @(posedge clk);

        // 이미지 적재 (4480 ~ 9599)
        i_load_en <= 1; @(posedge clk);
        for (i = 0; i < IMG_WORDS; i = i + 1) begin
            ld_waddr <= IMG_BASE + i; ld_wdata <= img[i]; ld_we <= 1; @(posedge clk);
        end
        ld_we <= 0; i_load_en <= 0; @(posedge clk);

        // 추론 시작
        i_start <= 1; @(posedge clk); i_start <= 0;
        $display("==== 640 full inference 시작 ====");

        // 추론 동안 출력 캡처/비교
        while (!o_all_done) begin
            @(posedge clk);
            check_one;
        end

        // 종료 직후 잔여 출력 플러시 (헤드 파이프라인 잔량)
        repeat (300) begin
            @(posedge clk);
            check_one;
        end

        $display("완료. layer_id=%0d  캡처=%0d/%0d  errs=%0d",
                 dut.u_core.layer_id, cap, N_AXI, errs);
        if (cap == N_AXI && errs == 0)
            $display(">>> [FULL INFER 640 PASS] 최종 출력까지 골든과 일치 <<<");
        else
            $display(">>> [FULL INFER 640 FAIL] (캡처 수 또는 값 불일치 - 체크포인트 TB로 발산 레이어 확인) <<<");
        $finish;
    end

    initial begin
        #(2_000_000 * 1000);   // 2 ms 상한 (32-bit 초과 방지: 곱으로 표현)
        $display("TIMEOUT lid=%0d cap=%0d", dut.u_core.layer_id, cap);
        $finish;
    end

endmodule
