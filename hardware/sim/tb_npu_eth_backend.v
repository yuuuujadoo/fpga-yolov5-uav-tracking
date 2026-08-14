`timescale 1ns / 1ps
//============================================================================
//  tb_npu_eth_backend.v   (자가검증 / self-checking)
//----------------------------------------------------------------------------
//  검증 항목
//   1) IMG 영역에 32-bit 워드 64개를 쓰면 2048-bit GFB 워드 1개가 정확히
//      조립되어 o_ld_we/o_ld_waddr/o_ld_wdata 로 방출되는가
//      (워드 2개분 = 128 write 로 waddr 0,1 두 번 방출되는지)
//   2) CTRL.START 로 o_start 펄스 / run 진입 / o_load_en=0 가 되는가
//   3) 추론중 BBox 3개를 받아 box_count=3, 메모리에 정확 저장되는가
//   4) all_done 후 done_sticky=1, o_load_en 복귀(=1)
//   5) STATUS read 가 {box_count,…,done,busy} 를 BRAM 타이밍으로 반환
//   6) RESULT read word0=count, 이후 3워드/박스가 정확히 반환
//
//  실행 (둘 중 하나)
//   - iverilog:  iverilog -o sim tb_npu_eth_backend.v npu_eth_backend.v && vvp sim
//   - Vivado xsim: xvlog npu_eth_backend.v tb_npu_eth_backend.v && xelab -R tb_npu_eth_backend
//============================================================================
module tb_npu_eth_backend;

    localparam GFB_WIDTH  = 2048;
    localparam ADDR_WIDTH = 14;
    localparam MAX_BOXES  = 256;
    localparam LANES      = GFB_WIDTH/32; // 64

    reg                    clk = 1'b0;
    reg                    rst = 1'b1;

    reg  [31:0]            i_cmd_addr  = 32'd0;
    reg  [31:0]            i_cmd_wdata = 32'd0;
    wire [31:0]            o_cmd_rdata;
    reg                    i_cmd_en    = 1'b0;
    reg  [3:0]             i_cmd_we    = 4'd0;

    wire                   o_load_en;
    wire [ADDR_WIDTH-1:0]  o_ld_waddr;
    wire                   o_ld_we;
    wire [GFB_WIDTH-1:0]   o_ld_wdata;

    wire                   o_start;
    reg                    i_all_done  = 1'b0;

    reg  [95:0]            i_bbox_tdata  = 96'd0;
    reg                    i_bbox_tvalid = 1'b0;
    wire                   o_bbox_tready;

    wire                   o_busy, o_done;
    wire [15:0]            o_box_count;

    integer errors = 0;

    //----------------------- DUT -----------------------
    npu_eth_backend #(
        .GFB_WIDTH(GFB_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .MAX_BOXES(MAX_BOXES)
    ) dut (
        .clk(clk), .rst(rst),
        .i_cmd_addr(i_cmd_addr), .i_cmd_wdata(i_cmd_wdata),
        .o_cmd_rdata(o_cmd_rdata), .i_cmd_en(i_cmd_en), .i_cmd_we(i_cmd_we),
        .o_load_en(o_load_en), .o_ld_waddr(o_ld_waddr),
        .o_ld_we(o_ld_we), .o_ld_wdata(o_ld_wdata),
        .o_start(o_start), .i_all_done(i_all_done),
        .i_bbox_tdata(i_bbox_tdata), .i_bbox_tvalid(i_bbox_tvalid),
        .o_bbox_tready(o_bbox_tready),
        .o_busy(o_busy), .o_done(o_done), .o_box_count(o_box_count)
    );

    always #5 clk = ~clk;   // 100 MHz

    //----------------------- 기대값 함수 -----------------------
    // IMG 워드 w, 레인 l 에 쓰는 32-bit 패턴
    function [31:0] img_pat;
        input integer w; input integer l;
        img_pat = 32'hA5000000 | (w[7:0] << 8) | l[7:0];
    endfunction

    //----------------------- 캡처: o_ld_we 방출 기록 -----------------------
    reg  [GFB_WIDTH-1:0] cap_wdata [0:7];
    reg  [ADDR_WIDTH-1:0] cap_waddr [0:7];
    integer cap_n = 0;
    always @(posedge clk) begin
        if (!rst && o_ld_we) begin
            cap_wdata[cap_n] <= o_ld_wdata;
            cap_waddr[cap_n] <= o_ld_waddr;
            cap_n <= cap_n + 1;
        end
    end

    //----------------------- 태스크: 32-bit write (엔드포인트 1클럭 스트로브) -----------------------
    task cmd_write;
        input [31:0] addr; input [31:0] data;
        begin
            @(negedge clk);
            i_cmd_addr  = addr;
            i_cmd_wdata = data;
            i_cmd_we    = 4'hf;
            i_cmd_en    = 1'b1;
            @(negedge clk);
            i_cmd_en    = 1'b0;
            i_cmd_we    = 4'd0;
        end
    endtask

    //----------------------- 태스크: 32-bit read (BRAM 타이밍: en->1클럭뒤 샘플) -----------------------
    task cmd_read;
        input  [31:0] addr;
        output [31:0] data;
        begin
            @(negedge clk);
            i_cmd_addr = addr;
            i_cmd_we   = 4'd0;
            i_cmd_en   = 1'b1;     // 이 클럭에서 is_read=1
            @(negedge clk);
            i_cmd_en   = 1'b0;     // 다음 클럭: o_cmd_rdata 유효
            @(negedge clk);
            data = o_cmd_rdata;
        end
    endtask

    //----------------------- 박스 주입 (ready 존중) -----------------------
    task push_box;
        input [95:0] b;
        begin
            @(negedge clk);
            i_bbox_tdata  = b;
            i_bbox_tvalid = 1'b1;
            // ready 가 1 이 될 때까지 대기 후 1클럭 핸드셰이크
            while (o_bbox_tready !== 1'b1) @(negedge clk);
            @(negedge clk);
            i_bbox_tvalid = 1'b0;
        end
    endtask

    //----------------------- 본문 -----------------------
    integer w, l;
    reg [31:0] rd;
    reg [GFB_WIDTH-1:0] exp_word;
    integer k;
    integer dl, shown;

    initial begin
        // 리셋
        repeat (4) @(negedge clk);
        rst = 1'b0;
        @(negedge clk);

        if (o_load_en !== 1'b1) begin
            $display("FAIL: reset 후 o_load_en 이 1 이 아님"); errors=errors+1;
        end

        //========= 1) 이미지 2 GFB 워드(=128 word) 쓰기 =========
        for (w = 0; w < 2; w = w + 1)
            for (l = 0; l < LANES; l = l + 1)
                cmd_write((w*LANES + l) << 2, img_pat(w,l)); // byte addr = (word*64+lane)*4

        // 방출 캡처가 끝나도록 settle 여유(타이밍 레이스 방지)
        repeat (4) @(negedge clk);
        if (cap_n !== 2) begin
            $display("FAIL: GFB 워드 방출 횟수 = %0d (기대 2)", cap_n); errors=errors+1;
        end else
            $display("PASS: GFB 워드 방출 2회");
        for (w = 0; w < 2; w = w + 1) begin
            exp_word = {GFB_WIDTH{1'b0}};
            for (l = 0; l < LANES; l = l + 1)
                exp_word[l*32 +: 32] = img_pat(w,l);
            if (cap_waddr[w] !== w[ADDR_WIDTH-1:0]) begin
                $display("FAIL: 워드%0d waddr=%0d (기대 %0d)", w, cap_waddr[w], w); errors=errors+1;
            end
            if (cap_wdata[w] !== exp_word) begin
                $display("FAIL: 워드%0d 데이터 불일치 -- 불일치 레인 진단:", w);
                errors = errors + 1;
                // 첫 8개 불일치 레인의 got/exp 를 출력 (원인 특정용)
                shown = 0;
                for (dl = 0; dl < LANES; dl = dl + 1)
                    if ((cap_wdata[w][dl*32 +: 32] !== img_pat(w,dl)) && (shown < 8)) begin
                        $display("   lane%0d: got=%08h  exp=%08h",
                                 dl, cap_wdata[w][dl*32 +: 32], img_pat(w,dl));
                        shown = shown + 1;
                    end
            end else
                $display("PASS: GFB 워드%0d 조립/주소 정확", w);
        end

        //========= 2) START =========
        cmd_write(32'h1000_0000, 32'h0000_0001);
        @(negedge clk);
        if (o_busy !== 1'b1)    begin $display("FAIL: START 후 busy 미설정"); errors=errors+1; end
        if (o_load_en !== 1'b0) begin $display("FAIL: 추론중 load_en 이 0 아님"); errors=errors+1; end
        else $display("PASS: START -> run 진입, load_en=0");

        //========= 3) BBox 3개 주입 =========
        push_box(96'h0000_0001_0000_0002_0000_0003);
        push_box(96'h0000_0011_0000_0012_0000_0013);
        push_box(96'h0000_0021_0000_0022_0000_0023);
        @(negedge clk);
        if (o_box_count !== 16'd3) begin
            $display("FAIL: box_count=%0d (기대 3)", o_box_count); errors=errors+1;
        end else $display("PASS: BBox 3개 수집");

        //========= 4) all_done =========
        @(negedge clk); i_all_done = 1'b1;
        @(negedge clk); i_all_done = 1'b0;
        @(negedge clk);
        if (o_done !== 1'b1)    begin $display("FAIL: done_sticky 미설정"); errors=errors+1; end
        if (o_load_en !== 1'b1) begin $display("FAIL: done 후 load_en 복귀 안됨"); errors=errors+1; end
        else $display("PASS: all_done -> done=1, load_en 복귀");

        //========= 5) STATUS read =========
        cmd_read(32'h2000_0000, rd);
        if (rd[0] !== 1'b0)  begin $display("FAIL: STATUS busy!=0"); errors=errors+1; end
        if (rd[1] !== 1'b1)  begin $display("FAIL: STATUS done!=1"); errors=errors+1; end
        if (rd[31:16] !== 16'd3) begin $display("FAIL: STATUS box_count=%0d", rd[31:16]); errors=errors+1; end
        else $display("PASS: STATUS read = {count=3, done=1, busy=0}");

        //========= 6) RESULT read =========
        cmd_read(32'h3000_0000, rd); // word0
        if (rd !== 32'd3) begin $display("FAIL: RESULT word0=%0d (기대 3)", rd); errors=errors+1; end
        else $display("PASS: RESULT word0=count=3");

        // box0
        cmd_read(32'h3000_0004, rd); if (rd!==32'h0000_0003) begin $display("FAIL: box0[31:0]=%h",rd);  errors=errors+1; end
        cmd_read(32'h3000_0008, rd); if (rd!==32'h0000_0002) begin $display("FAIL: box0[63:32]=%h",rd); errors=errors+1; end
        cmd_read(32'h3000_000C, rd); if (rd!==32'h0000_0001) begin $display("FAIL: box0[95:64]=%h",rd); errors=errors+1; end
        // box2 (인덱스 2) 검증
        cmd_read(32'h3000_001C, rd); if (rd!==32'h0000_0023) begin $display("FAIL: box2[31:0]=%h",rd);  errors=errors+1; end
        cmd_read(32'h3000_0020, rd); if (rd!==32'h0000_0022) begin $display("FAIL: box2[63:32]=%h",rd); errors=errors+1; end
        cmd_read(32'h3000_0024, rd); if (rd!==32'h0000_0021) begin $display("FAIL: box2[95:64]=%h",rd); errors=errors+1; end
        if (errors==0) $display("PASS: RESULT 박스 데이터 회수 정확");

        //========= 결과 =========
        $display("==================================================");
        if (errors == 0) $display(">>> ALL TESTS PASSED <<<");
        else             $display(">>> %0d 개 실패 <<<", errors);
        $display("==================================================");
        $finish;
    end

    // 워치독
    initial begin
        #200000;
        $display("FAIL: 타임아웃");
        $finish;
    end

endmodule
