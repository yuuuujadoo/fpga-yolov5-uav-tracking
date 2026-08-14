`timescale 1 ns / 1 ps

module mdio_phy_init #(
    parameter [4:0] PHY_ADDR = 5'd0,
    parameter integer CLK_DIV = 31,
    parameter integer RESET_HOLD_CYCLES = 1250000,
    parameter integer POST_RESET_CYCLES = 1250000,
    parameter integer PHY_RESET_SETTLE_CYCLES = 12500000
) (
    input  wire clk,
    input  wire rst,

    output wire phy_rst_n,
    output wire mdio_mdc,
    output wire mdio_mdio_o,
    output wire mdio_mdio_t,

    output reg  init_done,
    output reg  init_busy
);

    localparam [4:0] REG_BMCR       = 5'd0;
    localparam [4:0] REG_ANAR       = 5'd4;
    localparam [4:0] REG_GBCR       = 5'd9;

    localparam [15:0] BMCR_RESET    = 16'h8000;
    localparam [15:0] ANAR_DEFAULT  = 16'h01e1;
    localparam [15:0] GBCR_1000_ADV = 16'h0300;
    localparam [15:0] BMCR_AN_RESTART_1G_FULL = 16'h1340;

    localparam [2:0]
        ST_RESET_HOLD  = 3'd0,
        ST_POST_RESET  = 3'd1,
        ST_START_WRITE = 3'd2,
        ST_WRITE       = 3'd3,
        ST_WAIT        = 3'd4,
        ST_DONE        = 3'd5;

    reg [2:0] state_reg = ST_RESET_HOLD;
    reg [31:0] wait_count_reg = 32'd0;
    reg [1:0] cmd_index_reg = 2'd0;
    reg phy_rst_n_reg = 1'b0;

    reg mdc_reg = 1'b0;
    reg mdio_o_reg = 1'b1;
    reg mdio_t_reg = 1'b1;
    reg [31:0] clk_div_count_reg = 32'd0;
    reg [63:0] frame_shift_reg = 64'hffff_ffff_ffff_ffff;
    reg [6:0] bit_count_reg = 7'd0;
    reg frame_active_reg = 1'b0;
    reg frame_done_reg = 1'b0;

    assign phy_rst_n = phy_rst_n_reg;
    assign mdio_mdc = mdc_reg;
    assign mdio_mdio_o = mdio_o_reg;
    assign mdio_mdio_t = mdio_t_reg;

    function [4:0] cmd_reg_addr;
        input [1:0] index;
        begin
            case (index)
                2'd0: cmd_reg_addr = REG_BMCR;
                2'd1: cmd_reg_addr = REG_ANAR;
                2'd2: cmd_reg_addr = REG_GBCR;
                default: cmd_reg_addr = REG_BMCR;
            endcase
        end
    endfunction

    function [15:0] cmd_data;
        input [1:0] index;
        begin
            case (index)
                2'd0: cmd_data = BMCR_RESET;
                2'd1: cmd_data = ANAR_DEFAULT;
                2'd2: cmd_data = GBCR_1000_ADV;
                default: cmd_data = BMCR_AN_RESTART_1G_FULL;
            endcase
        end
    endfunction

    function [63:0] mdio_write_frame;
        input [4:0] phy_addr;
        input [4:0] reg_addr;
        input [15:0] data;
        begin
            mdio_write_frame = {32'hffff_ffff, 2'b01, 2'b01, phy_addr, reg_addr, 2'b10, data};
        end
    endfunction

    always @(posedge clk) begin
        frame_done_reg <= 1'b0;

        if (rst) begin
            mdc_reg <= 1'b0;
            mdio_o_reg <= 1'b1;
            mdio_t_reg <= 1'b1;
            clk_div_count_reg <= 32'd0;
            frame_shift_reg <= 64'hffff_ffff_ffff_ffff;
            bit_count_reg <= 7'd0;
            frame_active_reg <= 1'b0;
        end

        if (!rst && state_reg == ST_START_WRITE && !frame_active_reg) begin
            mdc_reg <= 1'b0;
            mdio_t_reg <= 1'b0;
            frame_shift_reg <= mdio_write_frame(PHY_ADDR, cmd_reg_addr(cmd_index_reg), cmd_data(cmd_index_reg));
            mdio_o_reg <= 1'b1;
            bit_count_reg <= 7'd0;
            clk_div_count_reg <= 32'd0;
            frame_active_reg <= 1'b1;
        end else if (!rst && frame_active_reg) begin
            if (clk_div_count_reg == CLK_DIV) begin
                clk_div_count_reg <= 32'd0;
                mdc_reg <= ~mdc_reg;

                if (mdc_reg) begin
                    if (bit_count_reg == 7'd63) begin
                        frame_active_reg <= 1'b0;
                        frame_done_reg <= 1'b1;
                        mdio_t_reg <= 1'b1;
                        mdio_o_reg <= 1'b1;
                        mdc_reg <= 1'b0;
                    end else begin
                        bit_count_reg <= bit_count_reg + 7'd1;
                        frame_shift_reg <= {frame_shift_reg[62:0], 1'b1};
                        mdio_o_reg <= frame_shift_reg[62];
                    end
                end
            end else begin
                clk_div_count_reg <= clk_div_count_reg + 32'd1;
            end
        end else if (!rst) begin
            mdc_reg <= 1'b0;
            mdio_t_reg <= 1'b1;
            mdio_o_reg <= 1'b1;
            clk_div_count_reg <= 32'd0;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state_reg <= ST_RESET_HOLD;
            wait_count_reg <= 32'd0;
            cmd_index_reg <= 2'd0;
            phy_rst_n_reg <= 1'b0;
            init_done <= 1'b0;
            init_busy <= 1'b1;
        end else begin
            case (state_reg)
                ST_RESET_HOLD: begin
                    phy_rst_n_reg <= 1'b0;
                    init_done <= 1'b0;
                    init_busy <= 1'b1;
                    if (wait_count_reg == RESET_HOLD_CYCLES-1) begin
                        wait_count_reg <= 32'd0;
                        state_reg <= ST_POST_RESET;
                    end else begin
                        wait_count_reg <= wait_count_reg + 32'd1;
                    end
                end

                ST_POST_RESET: begin
                    phy_rst_n_reg <= 1'b1;
                    if (wait_count_reg == POST_RESET_CYCLES-1) begin
                        wait_count_reg <= 32'd0;
                        cmd_index_reg <= 2'd0;
                        state_reg <= ST_START_WRITE;
                    end else begin
                        wait_count_reg <= wait_count_reg + 32'd1;
                    end
                end

                ST_START_WRITE: begin
                    state_reg <= ST_WRITE;
                end

                ST_WRITE: begin
                    if (frame_done_reg) begin
                        wait_count_reg <= 32'd0;
                        state_reg <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (cmd_index_reg == 2'd0) begin
                        if (wait_count_reg == PHY_RESET_SETTLE_CYCLES-1) begin
                            wait_count_reg <= 32'd0;
                            cmd_index_reg <= cmd_index_reg + 2'd1;
                            state_reg <= ST_START_WRITE;
                        end else begin
                            wait_count_reg <= wait_count_reg + 32'd1;
                        end
                    end else if (cmd_index_reg == 2'd3) begin
                        state_reg <= ST_DONE;
                    end else begin
                        cmd_index_reg <= cmd_index_reg + 2'd1;
                        state_reg <= ST_START_WRITE;
                    end
                end

                ST_DONE: begin
                    phy_rst_n_reg <= 1'b1;
                    init_done <= 1'b1;
                    init_busy <= 1'b0;
                end

                default: begin
                    state_reg <= ST_RESET_HOLD;
                end
            endcase
        end
    end

endmodule
