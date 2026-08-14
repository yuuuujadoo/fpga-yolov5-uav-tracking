`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// tb_cp_all_640.v  -  640x640 전체 추론 중 16개 체크포인트 GFB 를 골든과 대조
//   목적: 팀원의 DSP 절감(3x3 시분할) 수정본이 "기능적으로 동일"한지 검증.
//         첫 발산(틀어지는) 레이어를 정확히 찾아낸다.
//   원리: 입력 이미지를 GFB(IMG_BASE=4480)에 적재 -> 전체 추론 ->
//         미리 정한 16개 레이어가 끝나는 시점에 해당 GFB 영역을 읽어
//         골든(sl_full640_gold.mem)과 2048-bit 워드 단위로 비교.
//   재학습 불필요: 골든은 "정답 출력"이므로 가중치/양자화 .mem 변경 없이 그대로 사용.
//
//   필요 파일 (xsim 작업 디렉토리에 배치):
//     - lp_full640.mem        (= 프로젝트 layer_params.mem, L0 base=4480 패치본)
//     - sl_full640_img.mem    (640 입력 이미지, 5120 워드)
//     - sl_full640_gold.mem   (16개 체크포인트의 정답 GFB, 2048-bit 워드들)
//   DUT 소스: npu_top + (팀원 수정 4개 파일로 교체된) conv 데이터패스 전체.
//////////////////////////////////////////////////////////////////////////////////

module tb_cp_all_640;

    // ----------------------------------------------------------------------
    // [파라미터] 640 입력 규격
    //   IMG_WORDS=5120 (640행 x 8워드), IMG_BASE=4480 (=GFB_DEPTH 9600 - 5120)
    //   NUM_STEPS=55 (640 그래프의 instr 수; gen_full640.py len(ins) 와 일치해야 함)
    // ----------------------------------------------------------------------
    parameter IMG_WORDS = 5120;
    parameter IMG_BASE   = 4480;
    parameter NUM_STEPS  = 57;   // 640 그래프 = 57 스텝 (헤드 detect = lid 56). 55면 헤드 누락→bbox 0개

    // ----------------------------------------------------------------------
    // [클럭/리셋] 100MHz (주기 10ns). DUT 내부 동작 검증이므로 단일 클럭이면 충분.
    // ----------------------------------------------------------------------
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ----------------------------------------------------------------------
    // [DUT 인터페이스] npu_top 의 GFB 적재 포트 + 추론 시작/완료 + 96-bit 출력
    // ----------------------------------------------------------------------
    reg          i_load_en;          // GFB 적재 인에이블
    reg  [13:0]  ld_waddr;           // GFB 쓰기 주소 (14-bit)
    reg          ld_we;              // GFB 쓰기 인에이블
    reg  [2047:0] ld_wdata;          // GFB 쓰기 데이터 (2048-bit = 한 워드)
    reg          i_start;            // 추론 시작 펄스
    wire         o_all_done;         // 전체 추론 완료
    wire [95:0]  axi_data;           // 96-bit BBox 출력 (이 TB에선 미사용)
    wire         axi_valid;
    reg          axi_ready;

    // ----------------------------------------------------------------------
    // [DUT 인스턴스]
    //   NUM_STEPS 는 640 그래프 기준. m_axis_* 는 헤드 출력(여기선 항상 ready).
    // ----------------------------------------------------------------------
    npu_top #(.NUM_STEPS(NUM_STEPS)) dut (
        .clk(clk), .rst_n(rst_n),
        .i_load_en(i_load_en), .i_ld_waddr(ld_waddr), .i_ld_we(ld_we), .i_ld_wdata(ld_wdata),
        .i_start(i_start), .o_all_done(o_all_done),
        .m_axis_tdata(axi_data), .m_axis_tvalid(axi_valid), .m_axis_tready(axi_ready)
    );

    // ----------------------------------------------------------------------
    // [메모리 버퍼]
    //   img  : 입력 이미지 (5120 워드, 여유분 포함 8192)
    //   gold : 16개 체크포인트의 정답 GFB 워드들 (누적 길이; 여유분 포함)
    // ----------------------------------------------------------------------
    reg [2047:0] img  [0:8191];
    reg [2047:0] gold [0:28671];      // sl_full640_gold.mem = 27600 워드 (체크포인트 NW 합계). 여유 포함

    // ----------------------------------------------------------------------
    // [체크포인트 테이블] sl_full640_cp.txt 와 1:1 일치
    //   LID  : 이 레이어 instr id 가 끝나는 순간 비교 (= 해당 출력이 GFB 에 다 써진 시점)
    //   BASE : 그 결과가 저장된 GFB 시작 워드 주소
    //   NW   : 비교할 워드 수
    //   GOFF : sl_full640_gold.mem 내에서 이 체크포인트 데이터의 시작 오프셋(누적)
    // ----------------------------------------------------------------------
    integer LID  [0:15];
    integer BASE [0:15];
    integer NW   [0:15];
    integer GOFF [0:15];

    integer i, w, k;
    integer errs_cp, total_err;
    integer prev_lid;
    reg [2047:0] rdword;          // GFB 에서 읽은 워드
    reg [15:0]   done_mask;       // 이미 비교한 체크포인트 표시(중복 방지)

    // ----------------------------------------------------------------------
    // [GFB 읽기 함수] 시뮬레이션 전용 - GFB 내부 구조에 맞춰 직접 접근
    //   실제 GFB 구조 (global_feature_buffer.v):
    //     - npu_top 레벨에 직접 인스턴스: 경로 = dut.u_gfb
    //     - 단일 mem 배열이 아니라 128-bit 슬라이스 16개(NUM_SLICES=16)로 분할
    //     - 각 슬라이스는 generate 루프 BRAM_2D_SLICES[i] 안에서 다시
    //       ram_lower[0:8191] / ram_upper[...] 두 뱅크로 분할
    //     - 2048-bit 워드 = {slice15,...,slice0} (slice0 = 하위 128b)
    //   따라서 한 워드를 읽으려면 16개 슬라이스를 lower/upper 경계(8192)에 맞춰
    //   각각 읽어 합친다.  DEPTH_LOWER=8192 (GFB localparam 과 일치).
    // ----------------------------------------------------------------------
    localparam DEPTH_LOWER = 8192;

    // xsim 은 generate-for 인스턴스 배열에 "변수 인덱스"(BRAM_2D_SLICES[s]) 로는
    // 계층 참조를 못 한다(VRFC 10-2991). 따라서 16개 슬라이스를 "상수 인덱스"로
    // 풀어서(unroll) 접근한다. lower/upper 뱅크 선택은 한 함수 안에서 처리.
    //   slice i 의 128-bit  -> 최종 2048-bit 워드의 [i*128 +: 128]
    `define GFB_SLICE(i) ((a < DEPTH_LOWER) ? \
        dut.u_gfb.BRAM_2D_SLICES[i].ram_lower[a] : \
        dut.u_gfb.BRAM_2D_SLICES[i].ram_upper[a - DEPTH_LOWER])

    function [2047:0] gfb_read;
        input integer a;
        reg [2047:0] acc;
        begin
            acc[  0 +: 128] = `GFB_SLICE(0);
            acc[128 +: 128] = `GFB_SLICE(1);
            acc[256 +: 128] = `GFB_SLICE(2);
            acc[384 +: 128] = `GFB_SLICE(3);
            acc[512 +: 128] = `GFB_SLICE(4);
            acc[640 +: 128] = `GFB_SLICE(5);
            acc[768 +: 128] = `GFB_SLICE(6);
            acc[896 +: 128] = `GFB_SLICE(7);
            acc[1024 +: 128] = `GFB_SLICE(8);
            acc[1152 +: 128] = `GFB_SLICE(9);
            acc[1280 +: 128] = `GFB_SLICE(10);
            acc[1408 +: 128] = `GFB_SLICE(11);
            acc[1536 +: 128] = `GFB_SLICE(12);
            acc[1664 +: 128] = `GFB_SLICE(13);
            acc[1792 +: 128] = `GFB_SLICE(14);
            acc[1920 +: 128] = `GFB_SLICE(15);
            gfb_read = acc;
        end
    endfunction

    // ----------------------------------------------------------------------
    // [체크포인트 비교 태스크]
    //   ci 번째 체크포인트의 BASE..BASE+NW-1 워드를 GFB 에서 읽어
    //   gold[GOFF+ .. ] 와 비교, 불일치 워드 수를 누적.
    // ----------------------------------------------------------------------
    task compare_cp;
        input integer ci;
        begin
            errs_cp = 0;
            for (w = 0; w < NW[ci]; w = w + 1) begin
                rdword = gfb_read(BASE[ci] + w);
                if (rdword !== gold[GOFF[ci] + w]) begin
                    if (errs_cp < 4) begin
                        $display("    [word %0d] addr=%0d", w, BASE[ci] + w);
                        $display("       exp=%064X", gold[GOFF[ci] + w]);
                        $display("       act=%064X", rdword);
                    end
                    errs_cp = errs_cp + 1;
                end
            end
            if (errs_cp == 0)
                $display("  [CP PASS] lid=%0d base=%0d nw=%0d  (정확히 일치)", LID[ci], BASE[ci], NW[ci]);
            else
                $display("  [CP FAIL] lid=%0d base=%0d nw=%0d  불일치 워드 %0d/%0d",
                         LID[ci], BASE[ci], NW[ci], errs_cp, NW[ci]);
            total_err = total_err + errs_cp;
        end
    endtask

    // ----------------------------------------------------------------------
    // [메인 시퀀스]
    // ----------------------------------------------------------------------
    initial begin
        // --- 체크포인트 테이블 채우기 (sl_full640_cp.txt 그대로) ---
        LID[0]=0;  LID[1]=1;  LID[2]=8;  LID[3]=19; LID[4]=20; LID[5]=27; LID[6]=28; LID[7]=34;
        LID[8]=35; LID[9]=36; LID[10]=37;LID[11]=38;LID[12]=44;LID[13]=45;LID[14]=46;LID[15]=52;

        BASE[0]=0;   BASE[1]=6400;BASE[2]=3200;BASE[3]=3200;BASE[4]=0;   BASE[5]=800; BASE[6]=4800;BASE[7]=1600;
        BASE[8]=0;   BASE[9]=6400;BASE[10]=4800;BASE[11]=0; BASE[12]=0;  BASE[13]=2400;BASE[14]=1600;BASE[15]=1600;

        NW[0]=6400;  NW[1]=3200; NW[2]=3200; NW[3]=1600; NW[4]=800;  NW[5]=800;  NW[6]=400;  NW[7]=1600;
        NW[8]=800;   NW[9]=400;  NW[10]=1600;NW[11]=3200;NW[12]=1600;NW[13]=400; NW[14]=800; NW[15]=800;

        // GOFF 누적 (sl_full640_gold.mem 내 위치). cp.txt 의 goff 와 동일하게 산출.
        GOFF[0] = 0;
        for (k = 1; k < 16; k = k + 1) GOFF[k] = GOFF[k-1] + NW[k-1];

        // --- 초기화 ---
        i_load_en = 0; ld_we = 0; i_start = 0; axi_ready = 1;
        total_err = 0; done_mask = 0; prev_lid = -1;

        // --- 골든/입력/레이어파라미터 로드 ---
        //  param_rom 경로는 프로젝트의 parameter_rom_array 인스턴스 경로에 맞춤.
        $readmemh("lp_full640.mem",     dut.u_core.u_prom.param_rom);
        $readmemh("sl_full640_img.mem", img);
        $readmemh("sl_full640_gold.mem", gold);

        // --- 리셋 해제 ---
        #20 rst_n = 1; @(posedge clk);

        // --- 이미지 GFB 적재 (4480 ~ 9599) ---
        i_load_en <= 1; @(posedge clk);
        for (i = 0; i < IMG_WORDS; i = i + 1) begin
            ld_waddr <= IMG_BASE + i;
            ld_wdata <= img[i];
            ld_we    <= 1;
            @(posedge clk);
        end
        ld_we <= 0; i_load_en <= 0; @(posedge clk);

        // --- 추론 시작 ---
        i_start <= 1; @(posedge clk); i_start <= 0;
        $display("==== 640 inference 시작 (DSP 절감 수정본 검증) ====");

        // --- 레이어 진행을 감시하며, 각 체크포인트 LID 가 "방금 끝난" 순간 비교 ---
        //   layer_id 가 LID[ci] 보다 커지는 순간 = 그 레이어 출력이 GFB 에 확정된 시점.
        while (!o_all_done) begin
            @(posedge clk);
            // layer_id 가 바뀐 경우만 검사 (매 클럭 비교 방지)
            if (dut.u_core.layer_id !== prev_lid) begin
                for (k = 0; k < 16; k = k + 1) begin
                    // 체크포인트 레이어가 끝났고(현재 layer_id > LID[k]) 아직 비교 안 했으면 비교
                    if (!done_mask[k] && (dut.u_core.layer_id > LID[k])) begin
                        $display("[검사] 체크포인트 #%0d (lid=%0d 직후)", k, LID[k]);
                        compare_cp(k);
                        done_mask[k] = 1'b1;
                    end
                end
                prev_lid = dut.u_core.layer_id;
            end
        end

        // --- 추론이 끝났는데 아직 비교 못 한 체크포인트(마지막 레이어들) 처리 ---
        for (k = 0; k < 16; k = k + 1) begin
            if (!done_mask[k]) begin
                $display("[검사-종료후] 체크포인트 #%0d (lid=%0d)", k, LID[k]);
                compare_cp(k);
                done_mask[k] = 1'b1;
            end
        end

        // --- 최종 판정 ---
        $display("==================================================");
        if (total_err == 0)
            $display(">>> [CP ALL PASS] 16개 체크포인트 전부 골든과 일치 - DSP 절감 수정본 기능 동일 <<<");
        else
            $display(">>> [CP FAIL] 총 불일치 워드 %0d개. 위 로그에서 '첫 FAIL 레이어'가 발산 지점 <<<", total_err);
        $finish;
    end

    // ----------------------------------------------------------------------
    // [타임아웃 보호] 무한 루프 방지
    // ----------------------------------------------------------------------
    initial begin
        // 32-bit 초과 지연은 분할(곱)로 표현해야 xsim 경고/오작동 방지
        #(2_000_000 * 1000);   // 2,000,000,000 ns = 2 ms 시뮬레이션 시간 상한
        $display("TIMEOUT: layer_id=%0d (추론이 끝나지 않음 - done 신호/핸드셰이크 확인 필요)",
                 dut.u_core.layer_id);
        $finish;
    end

endmodule
