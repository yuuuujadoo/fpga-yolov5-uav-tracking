`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: sequencer_fsm (Top 제어 전담 FSM - 패턴 A)
//
// 역할: parameter_rom 스텝을 순회하며, op 종류에 따라 active_engine 을 선택하고
//   해당 엔진만 start, done 대기 후 다음 스텝 전진. 전체 끝나면 o_all_done.
//   엔진/자원 인스턴스화는 하지 않음 (top_wrapper 가 배선). FSM 은 제어 신호만.
//
// op 종류 -> active_engine (명령어 비트맵 플래그):
//   is_detect=1                       -> HEAD
//   is_pool=1                         -> POOL
//   is_concat|is_byp|is_upsample=1    -> VEC
//   그 외 (conv 1x1/3x3)              -> CONV
//
// 엔진 제어: active 에 따라 해당 start 만 1펄스, 해당 done 대기.
// weight/param 주소: conv 일 때만 생성 (conv_engine o_req_grp/fold 기반).
//////////////////////////////////////////////////////////////////////////////////
module sequencer_fsm #(
    parameter NUM_STEPS = 53,
    parameter ADDR_WIDTH = 14
)(
    input  wire clk, rst_n,
    input  wire i_start,
    output reg  o_all_done,

    // parameter_rom 인터페이스
    output reg  [6:0]  o_layer_id,
    output reg         o_prom_en,
    input  wire        i_is_detect, i_is_upsample, i_is_concat, i_is_byp, i_is_pool, i_is_1x1,
    input  wire [9:0]  i_total_ch_in, i_out_ch,
    input  wire [13:0] i_weight_addr, i_raddr_A,
    input  wire [9:0]  i_param_addr,

    // active_engine 선택 (top_wrapper 의 mux 제어)
    output reg  [1:0]  o_active,    // 0=CONV,1=POOL,2=VEC,3=HEAD
    // 엔진 start (active 에 따라 1개만 펄스)
    output reg         o_conv_start, o_pool_start, o_vec_start, o_head_start,
    // 엔진 done (top_wrapper 가 active 엔진 done 을 묶어 전달)
    input  wire        i_conv_done, i_pool_done, i_vec_done, i_head_done,

    // conv weight/param 주소 생성 (conv_engine 요청 기반)
    input  wire [4:0]  i_conv_req_grp, i_conv_req_fold,
    output wire [13:0] o_weight_addr,
    output wire [9:0]  o_param_addr,
    // P3/P4 판별 (head)
    output reg         o_is_p4
);
    localparam CONV=0, POOL=1, VEC=2, HEAD=3;

    // op 종류 판별 (조합)
    //   Detect head 는 2단계: (1) 1x1 conv(->18ch, silu off) = is_detect&is_1x1 -> CONV
    //                          (2) bbox decode = is_detect&!is_1x1 -> HEAD
    //   silu_en=~is_detect 로 (1)단계 conv 도 silu off 유지 (golden 일치).
    wire op_head = i_is_detect & ~i_is_1x1;
    wire op_pool = i_is_pool;
    wire op_vec  = i_is_concat | i_is_byp | i_is_upsample;
    // 나머지는 conv
    wire [1:0] op_active = op_head ? HEAD : op_pool ? POOL : op_vec ? VEC : CONV;

    // weight/param 주소 (conv 전용)
    wire [4:0] in_folds = i_total_ch_in[9:4];
    wire [4:0] fold_clamped = (i_conv_req_fold < in_folds) ? i_conv_req_fold : (in_folds-1);
    assign o_weight_addr = i_weight_addr + i_conv_req_grp*in_folds + fold_clamped;
    assign o_param_addr  = i_param_addr  + i_conv_req_grp;

    // 현재 active 엔진의 done
    reg [1:0] cur_active;
    wire cur_done = (cur_active==CONV) ? i_conv_done :
                    (cur_active==POOL) ? i_pool_done :
                    (cur_active==VEC)  ? i_vec_done  : i_head_done;

    localparam S_IDLE=0, S_FETCH=1, S_DECODE=2, S_RUN=3, S_NEXT=4, S_DONE=5;
    reg [2:0] state;

    always @(posedge clk) begin
        if(!rst_n) begin
            state<=S_IDLE; o_all_done<=0; o_layer_id<=0; o_prom_en<=0;
            o_conv_start<=0; o_pool_start<=0; o_vec_start<=0; o_head_start<=0;
            o_active<=CONV; cur_active<=CONV; o_is_p4<=0;
        end else begin
            o_all_done<=0; o_prom_en<=0;
            o_conv_start<=0; o_pool_start<=0; o_vec_start<=0; o_head_start<=0;
            case(state)
                S_IDLE: if(i_start) begin
                    o_layer_id<=0; o_prom_en<=1; state<=S_FETCH;
                end
                S_FETCH: begin
                    o_prom_en<=1;       // parameter_rom[layer_id] 읽기 (1클럭 latency)
                    state<=S_DECODE;
                end
                S_DECODE: begin
                    // 디코드 신호 유효. op 판별 -> active 선택 -> 해당 엔진 start.
                    o_active   <= op_active;
                    cur_active <= op_active;
                    case(op_active)
                        CONV: o_conv_start<=1;
                        POOL: o_pool_start<=1;
                        VEC:  o_vec_start<=1;
                        HEAD: begin o_head_start<=1; o_is_p4<= (i_raddr_A[0]); end // P4 판별 임시
                    endcase
                    state<=S_RUN;
                end
                S_RUN: begin
                    if(cur_done) state<=S_NEXT;
                end
                S_NEXT: begin
                    if(o_layer_id + 1 < NUM_STEPS) begin
                        o_layer_id<=o_layer_id+1; o_prom_en<=1; state<=S_FETCH;
                    end else state<=S_DONE;
                end
                S_DONE: begin o_all_done<=1; state<=S_IDLE; end
            endcase
        end
    end
endmodule
