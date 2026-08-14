`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: vec_engine (CONCAT / ADDER / UPSAMPLE)
//
// 채널 비혼합. 두 입력소스(raddr_A/B). GFB 빽빽 패킹(1워드=16fold, 전역fold=p*INF+f).
// 주소 재배치(concat/upsample) + residual_adder(adder).
//
// 검증 단순화: 한 픽셀의 fold 들이 한 GFB 워드 안에 정렬(p*INF 가 16의 배수 또는
//   INF 가 16의 약수)된다고 가정. 작은 텐서 검증에 충분. (일반화는 후속.)
//
// 처리: 픽셀 단위로 A(,B) 워드를 읽고, fold 들을 순차로 출력 전역fold 슬롯에 기록.
//   출력 워드는 누적하여 슬롯15(또는 마지막) 시 GFB write.
//////////////////////////////////////////////////////////////////////////////////
module vec_engine #(
    parameter DATA_WIDTH=8, parameter T0=16, parameter FOLD_CH=16,
    parameter GFB_WIDTH=2048, parameter ADDR_WIDTH=14
)(
    input  wire clk, rst_n,
    input  wire        i_start,
    output reg         o_done,
    output reg         o_busy,
    input  wire        i_is_concat, i_is_byp, i_is_upsample,
    input  wire [13:0] i_raddr_A, i_raddr_B, i_waddr,
    input  wire [9:0]  i_ch_a, i_ch_b,
    input  wire [19:0] i_target_pixel,
    input  wire [10:0] i_img_width,
    output reg  [ADDR_WIDTH-1:0] o_gfb_raddr,
    output reg         o_gfb_re,
    input  wire [GFB_WIDTH-1:0]  i_gfb_rdata,
    output reg  [ADDR_WIDTH-1:0] o_gfb_waddr,
    output reg         o_gfb_we,
    output reg  [GFB_WIDTH-1:0]  o_gfb_wdata
);
    wire [4:0] inf_a = i_ch_a[9:4];
    wire [4:0] inf_b = i_ch_b[9:4];
    wire [4:0] inf_o = i_is_concat ? (inf_a+inf_b) : inf_a;
    wire [10:0] ow = i_img_width << 1;

    // residual_adder (조합 아닌 1클럭 지연 모듈)
    reg  [T0*DATA_WIDTH-1:0] ra_main, ra_res; reg ra_vin;
    wire [T0*DATA_WIDTH-1:0] ra_sum; wire ra_vout;
    residual_adder #(.T0(T0),.DATA_WIDTH(DATA_WIDTH)) u_adder(
        .clk(clk),.rst_n(rst_n),.data_main_in(ra_main),.valid_in(ra_vin),
        .last_fold_in(1'b0),.px_in(11'd0),.py_in(11'd0),.data_res_in(ra_res),
        .ready_in(1'b1),.ready_out(),.sum_data_out(ra_sum),
        .valid_out(ra_vout),.last_fold_out(),.px_out(),.py_out());

    // 출력 워드 누적 write (공통)
    reg [GFB_WIDTH-1:0] out_word, out_word_n;
    reg        wr_en; reg [19:0] wr_gfold; reg [127:0] wr_data;
    reg        wr_force; reg [13:0] wr_force_addr;
    integer sw;
    always @(posedge clk) begin
        if(!rst_n) begin out_word<=0; o_gfb_we<=0; o_gfb_waddr<=0; o_gfb_wdata<=0; end
        else begin
            o_gfb_we<=0;
            if(state==S_IDLE && i_start) out_word<=0;
            else if(wr_force) begin o_gfb_we<=1; o_gfb_waddr<=wr_force_addr; o_gfb_wdata<=out_word; end
            else if(wr_en) begin
                sw = wr_gfold % 16;
                out_word_n=out_word; out_word_n[sw*128 +: 128]=wr_data; out_word<=out_word_n;
                if(sw==15) begin o_gfb_we<=1; o_gfb_waddr<=i_waddr+(wr_gfold/16); o_gfb_wdata<=out_word_n; end
            end
        end
    end

    localparam S_IDLE=0, S_CALC=1, S_RA=2, S_RAW=3, S_GA=4,
               S_RB=5, S_RBW=6, S_GB=7, S_ADDW=8, S_WR=9, S_FLUSHW=10, S_DONE=11;
    reg [3:0] state;

    // ===== 메인 FSM (fold 단위 read, 워드경계 일반 처리) =====
    // 출력을 전역 출력fold 순서로 생성. 각 출력fold 마다 필요한 입력fold 를 read.
    reg [19:0] of_cnt;        // 출력 전역fold 카운터 (0..out_total_folds-1)
    reg [19:0] out_total;     // 총 출력 fold
    reg [4:0]  flush_cnt;
    reg [127:0] ga, gb;       // 읽은 입력 fold 데이터
    reg [19:0] cur_p, cur_f;  // 현재 출력픽셀, 픽셀내 출력fold
    reg [19:0] rd_gf;         // 읽을 입력 전역fold (임시)
    reg [19:0] inpix;         // upsample 입력픽셀
    // [TIMING] upsample 의 cur_p/ow, cur_p%ow (가변 나눗셈) 제거용 출력 좌표 증분 카운터.
    //   cur_p = oy*ow + ox 가 항상 성립 -> oy=cur_p/ow, ox=cur_p%ow 와 비트동일.
    reg [10:0] ox, oy;

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE; o_done<=0; o_busy<=0; o_gfb_re<=0;
            of_cnt<=0; cur_p<=0; cur_f<=0; flush_cnt<=0; ra_vin<=0; wr_en<=0;
        end else begin
            o_done<=0; o_gfb_re<=0; ra_vin<=0; wr_en<=0; wr_force<=0;
            case(state)
                S_IDLE: if(i_start) begin
                    o_busy<=1; of_cnt<=0; cur_p<=0; cur_f<=0; flush_cnt<=0;
                    ox<=0; oy<=0;   // [TIMING] upsample 출력좌표 카운터 초기화
                    out_total <= i_is_upsample ? (i_target_pixel*4*inf_a) :
                                 i_is_concat   ? (i_target_pixel*inf_o) :
                                                 (i_target_pixel*inf_a);
                    state<=S_CALC;
                end
                // 현재 출력fold(of_cnt) 의 픽셀/픽셀내fold 분해 -> 읽을 입력fold 결정
                S_CALC: begin
                    state<=S_RA;
                end
                // ---- 입력 A fold read ----
                S_RA: begin
                    // 읽을 A 전역fold rd_gf 계산
                    if(i_is_upsample) begin
                        // [TIMING] (cur_p/ow, cur_p%ow) 나눗셈 제거 -> oy,ox 카운터 사용. 입력픽셀=(oy/2,ox/2)
                        rd_gf = (((oy)>>1)*i_img_width + ((ox)>>1))*inf_a + cur_f;
                    end else if(i_is_concat) begin
                        if(cur_f < inf_a) rd_gf = cur_p*inf_a + cur_f;          // A
                        else              rd_gf = cur_p*inf_b + (cur_f-inf_a);  // B (S_RB 에서)
                    end else begin // adder
                        rd_gf = cur_p*inf_a + cur_f;
                    end
                    // concat 의 B 영역이면 A read 건너뜀
                    if(i_is_concat && cur_f>=inf_a) begin state<=S_RB; end
                    else begin
                        o_gfb_raddr<= i_raddr_A + rd_gf/16; o_gfb_re<=1;
                        rd_slot<= rd_gf%16; state<=S_RAW;
                    end
                end
                S_RAW: state<=S_GA;
                S_GA: begin
                    gat<= i_gfb_rdata[rd_slot*128 +: 128];
                    if(i_is_byp) state<=S_RB;       // adder 는 B 도 읽음
                    else state<=S_WR;               // concat(A영역)/upsample 은 바로 write
                end
                // ---- 입력 B fold read (concat B영역 / adder) ----
                S_RB: begin
                    if(i_is_concat) rd_gf = cur_p*inf_b + (cur_f-inf_a);
                    else            rd_gf = cur_p*inf_a + cur_f;   // adder: B 같은 위치
                    o_gfb_raddr<= i_raddr_B + rd_gf/16; o_gfb_re<=1;
                    rd_slot<= rd_gf%16; state<=S_RBW;
                end
                S_RBW: state<=S_GB;
                S_GB: begin
                    gbt<= i_gfb_rdata[rd_slot*128 +: 128];
                    if(i_is_byp) begin
                        ra_main<= gat; ra_res<= i_gfb_rdata[rd_slot*128 +: 128]; ra_vin<=1;
                        state<=S_ADDW;
                    end else state<=S_WR;   // concat B
                end
                S_ADDW: begin
                    if(ra_vout) begin wr_dat<= ra_sum; state<=S_WR; end
                end
                // ---- 출력 fold write (빽빽 누적) ----
                S_WR: begin
                    wr_en<=1; wr_gfold<= of_cnt;
                    if(i_is_byp)        wr_data<= wr_dat;
                    else if(i_is_concat && cur_f>=inf_a) wr_data<= gbt;
                    else                wr_data<= gat;
                    // 다음 출력fold
                    of_cnt<=of_cnt+1;
                    // 픽셀/픽셀내fold 갱신
                    if(cur_f+1 >= (i_is_concat?inf_o:inf_a)) begin
                        cur_f<=0; cur_p<=cur_p+1;
                        // [TIMING] 출력좌표 증분 (cur_p=oy*ow+ox 유지)
                        if(ox+1>=ow) begin ox<=0; oy<=oy+1; end else ox<=ox+1;
                    end else cur_f<=cur_f+1;
                    if(of_cnt+1 >= out_total) state<=S_FLUSHW;
                    else state<=S_CALC;
                end
                S_FLUSHW: begin
                    // 마지막 미완성 워드 flush
                    if(flush_cnt==0 && (out_total%16!=0)) begin
                        wr_force<=1; wr_force_addr<= i_waddr + ((out_total-1)/16);
                    end
                    if(flush_cnt>=4) state<=S_DONE; else flush_cnt<=flush_cnt+1;
                end
                S_DONE: begin o_done<=1; o_busy<=0; state<=S_IDLE; end
            endcase
        end
    end
    reg [4:0] rd_slot; reg [127:0] gat, gbt, wr_dat;

endmodule
