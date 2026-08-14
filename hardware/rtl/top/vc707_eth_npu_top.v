`timescale 1 ns / 1 ps

module vc707_eth_npu_top #(
    parameter real    APP_CLK_DIV    = 44.000, // MMCM CLKOUT0 divide: 10=100MHz, 12.5=80, 14=71.43, 16=62.5, 20=50MHz
    parameter integer IMG_WORDS_2048 = 64,      // number of 2048-bit GFB words the input image occupies (SET to real value)
    parameter integer MAX_BOXES      = 256      // max BBox to collect
) (
    input  wire sys_diff_clock_clk_p,
    input  wire sys_diff_clock_clk_n,
    input  wire reset,

    input  wire rxp,
    input  wire rxn,
    output wire txp,
    output wire txn,
    input  wire gtrefclk_p,
    input  wire gtrefclk_n,

    output wire mdio_mdc,
    output wire mdio_mdio_io,
    output wire phy_rst_n
);

    localparam [47:0] LOCAL_MAC = 48'h02_00_00_00_00_01;
    localparam [31:0] LOCAL_IP = {8'd192, 8'd168, 8'd1, 8'd10};
    localparam [31:0] GATEWAY_IP = {8'd192, 8'd168, 8'd1, 8'd1};
    localparam [31:0] SUBNET_MASK = {8'd255, 8'd255, 8'd255, 8'd0};
    localparam integer BRAM_WORDS = 4096;

    wire sys_clk_ibuf;
    wire sys_clk_bufg;

    IBUFDS #(
        .DIFF_TERM("TRUE"),
        .IBUF_LOW_PWR("FALSE")
    ) u_sys_clk_ibuf (
        .I(sys_diff_clock_clk_p),
        .IB(sys_diff_clock_clk_n),
        .O(sys_clk_ibuf)
    );

    BUFG u_sys_clk_bufg (
        .I(sys_clk_ibuf),
        .O(sys_clk_bufg)
    );

    wire app_clk_100_mmcm;
    wire app_clk_100;
    wire app_clk_fb;
    wire app_clk_fb_bufg;
    wire app_mmcm_locked;
    wire app_rst;

    MMCME2_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKIN1_PERIOD(5.000),
        .CLKFBOUT_MULT_F(5.000),
        .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(APP_CLK_DIV),
        .CLKOUT0_PHASE(0.000),
        .CLKOUT0_DUTY_CYCLE(0.500)
    ) u_app_mmcm (
        .CLKIN1(sys_clk_bufg),
        .CLKFBIN(app_clk_fb_bufg),
        .RST(reset),
        .PWRDWN(1'b0),
        .CLKFBOUT(app_clk_fb),
        .CLKFBOUTB(),
        .CLKOUT0(app_clk_100_mmcm),
        .CLKOUT0B(),
        .CLKOUT1(),
        .CLKOUT1B(),
        .CLKOUT2(),
        .CLKOUT2B(),
        .CLKOUT3(),
        .CLKOUT3B(),
        .CLKOUT4(),
        .CLKOUT5(),
        .CLKOUT6(),
        .LOCKED(app_mmcm_locked)
    );

    BUFG u_app_clk_fb_bufg (
        .I(app_clk_fb),
        .O(app_clk_fb_bufg)
    );

    BUFG u_app_clk_bufg (
        .I(app_clk_100_mmcm),
        .O(app_clk_100)
    );

    wire app_rst_raw = reset | ~app_mmcm_locked;
    reset_sync u_rst_sync_app  (.i_clk(app_clk_100), .i_rst_async(app_rst_raw), .o_rst_sync(app_rst));

    wire        eth_clk_125;
    wire        pcs_resetdone;
    wire        pcs_pma_reset_out;
    wire        pcs_mmcm_locked;
    wire        pcs_sgmii_clk_en;
    wire [7:0]  pcs_gmii_txd;
    wire        pcs_gmii_tx_en;
    wire        pcs_gmii_tx_er;
    wire [7:0]  pcs_gmii_rxd;
    wire        pcs_gmii_rx_dv;
    wire        pcs_gmii_rx_er;
    wire        pcs_gmii_isolate;
    wire        pcs_an_interrupt;
    wire [15:0] pcs_status_vector;
    wire [1:0]  pcs_status_speed = pcs_status_vector[11:10];
    wire [4:0]  pcs_config_vector;
    wire [15:0] pcs_an_adv_config_vector;
    wire        pcs_gmii_rst;

    wire unused_pcs_sgmii_clk_r;
    wire unused_pcs_sgmii_clk_f;
    wire unused_gtrefclk_out;
    wire unused_gtrefclk_bufg_out;
    wire unused_userclk_out;
    wire unused_rxuserclk_out;
    wire unused_rxuserclk2_out;
    wire unused_gt0_qplloutclk_out;
    wire unused_gt0_qplloutrefclk_out;
    wire unused_gmii_isolate = pcs_gmii_isolate;
    wire unused_an_interrupt = pcs_an_interrupt;

    assign pcs_config_vector[4] = 1'b1;
    assign pcs_config_vector[3] = 1'b0;
    assign pcs_config_vector[2] = 1'b0;
    assign pcs_config_vector[1] = 1'b0;
    assign pcs_config_vector[0] = 1'b0;

    assign pcs_an_adv_config_vector[15]    = 1'b1;
    assign pcs_an_adv_config_vector[14]    = 1'b1;
    assign pcs_an_adv_config_vector[13:12] = 2'b01;
    assign pcs_an_adv_config_vector[11:10] = 2'b10;
    assign pcs_an_adv_config_vector[9:1]   = 9'd0;
    assign pcs_an_adv_config_vector[0]     = 1'b1;

    wire pcs_gmii_rst_comb = reset | ~pcs_resetdone | pcs_pma_reset_out | ~pcs_mmcm_locked;
    (* ASYNC_REG = "TRUE" *) reg [1:0] r_gmii_rst_pipe = 2'b11;
    always @(posedge sys_clk_bufg) r_gmii_rst_pipe <= {r_gmii_rst_pipe[0], pcs_gmii_rst_comb};
    wire pcs_gmii_rst_raw = r_gmii_rst_pipe[1];
    reset_sync u_rst_sync_eth  (.i_clk(eth_clk_125), .i_rst_async(pcs_gmii_rst_raw), .o_rst_sync(pcs_gmii_rst));

    gig_ethernet_pcs_pma_0 u_pcs_pma (
        .gtrefclk_p(gtrefclk_p),
        .gtrefclk_n(gtrefclk_n),
        .gtrefclk_out(unused_gtrefclk_out),
        .gtrefclk_bufg_out(unused_gtrefclk_bufg_out),
        .txn(txn),
        .txp(txp),
        .rxn(rxn),
        .rxp(rxp),
        .independent_clock_bufg(sys_clk_bufg),
        .userclk_out(unused_userclk_out),
        .userclk2_out(eth_clk_125),
        .rxuserclk_out(unused_rxuserclk_out),
        .rxuserclk2_out(unused_rxuserclk2_out),
        .resetdone(pcs_resetdone),
        .pma_reset_out(pcs_pma_reset_out),
        .mmcm_locked_out(pcs_mmcm_locked),
        .sgmii_clk_r(unused_pcs_sgmii_clk_r),
        .sgmii_clk_f(unused_pcs_sgmii_clk_f),
        .sgmii_clk_en(pcs_sgmii_clk_en),
        .gmii_txd(pcs_gmii_txd),
        .gmii_tx_en(pcs_gmii_tx_en),
        .gmii_tx_er(pcs_gmii_tx_er),
        .gmii_rxd(pcs_gmii_rxd),
        .gmii_rx_dv(pcs_gmii_rx_dv),
        .gmii_rx_er(pcs_gmii_rx_er),
        .gmii_isolate(pcs_gmii_isolate),
        .configuration_vector(pcs_config_vector),
        .an_interrupt(pcs_an_interrupt),
        .an_adv_config_vector(pcs_an_adv_config_vector),
        .an_restart_config(1'b0),
        .speed_is_10_100(pcs_status_speed != 2'b10),
        .speed_is_100(pcs_status_speed == 2'b01),
        .status_vector(pcs_status_vector),
        .reset(reset),
        .signal_detect(1'b1),
        .gt0_qplloutclk_out(unused_gt0_qplloutclk_out),
        .gt0_qplloutrefclk_out(unused_gt0_qplloutrefclk_out)
    );

    wire mdio_mdio_o;
    wire mdio_mdio_t;
    wire phy_init_done;
    wire phy_init_busy;

    OBUFT u_mdio_obuf (
        .I(mdio_mdio_o),
        .T(mdio_mdio_t),
        .O(mdio_mdio_io)
    );

    mdio_phy_init #(
        .PHY_ADDR(5'd7)
    ) u_mdio_phy_init (
        .clk(app_clk_100),
        .rst(app_rst),
        .phy_rst_n(phy_rst_n),
        .mdio_mdc(mdio_mdc),
        .mdio_mdio_o(mdio_mdio_o),
        .mdio_mdio_t(mdio_mdio_t),
        .init_done(phy_init_done),
        .init_busy(phy_init_busy)
    );

    wire [7:0] rx_axis_mac_tdata;
    wire       rx_axis_mac_tvalid;
    wire       rx_axis_mac_tready;
    wire       rx_axis_mac_tlast;
    wire       rx_axis_mac_tuser;
    wire [0:0] rx_axis_mac_tkeep;

    wire [7:0] rx_axis_app_tdata;
    wire       rx_axis_app_tvalid;
    wire       rx_axis_app_tready;
    wire       rx_axis_app_tlast;
    wire       rx_axis_app_tuser;
    wire [0:0] rx_axis_app_tkeep;

    wire [7:0] tx_axis_app_tdata;
    wire       tx_axis_app_tvalid;
    wire       tx_axis_app_tready;
    wire       tx_axis_app_tlast;
    wire       tx_axis_app_tuser;
    wire [0:0] tx_axis_app_tkeep;

    wire [7:0] tx_axis_mac_tdata;
    wire       tx_axis_mac_tvalid;
    wire       tx_axis_mac_tready;
    wire       tx_axis_mac_tlast;
    wire       tx_axis_mac_tuser;
    wire [0:0] tx_axis_mac_tkeep;

    eth_mac_1g_fifo #(
        .AXIS_DATA_WIDTH(8),
        .ENABLE_PADDING(1),
        .MIN_FRAME_LENGTH(64),
        .TX_FIFO_DEPTH(4096),
        .TX_FRAME_FIFO(1),
        .RX_FIFO_DEPTH(4096),
        .RX_FRAME_FIFO(1)
    ) u_eth_mac (
        .rx_clk(eth_clk_125),
        .rx_rst(pcs_gmii_rst),
        .tx_clk(eth_clk_125),
        .tx_rst(pcs_gmii_rst),
        .logic_clk(eth_clk_125),
        .logic_rst(pcs_gmii_rst),
        .tx_axis_tdata(tx_axis_mac_tdata),
        .tx_axis_tkeep(tx_axis_mac_tkeep),
        .tx_axis_tvalid(tx_axis_mac_tvalid),
        .tx_axis_tready(tx_axis_mac_tready),
        .tx_axis_tlast(tx_axis_mac_tlast),
        .tx_axis_tuser(tx_axis_mac_tuser),
        .rx_axis_tdata(rx_axis_mac_tdata),
        .rx_axis_tkeep(rx_axis_mac_tkeep),
        .rx_axis_tvalid(rx_axis_mac_tvalid),
        .rx_axis_tready(rx_axis_mac_tready),
        .rx_axis_tlast(rx_axis_mac_tlast),
        .rx_axis_tuser(rx_axis_mac_tuser),
        .gmii_rxd(pcs_gmii_rxd),
        .gmii_rx_dv(pcs_gmii_rx_dv),
        .gmii_rx_er(pcs_gmii_rx_er),
        .gmii_txd(pcs_gmii_txd),
        .gmii_tx_en(pcs_gmii_tx_en),
        .gmii_tx_er(pcs_gmii_tx_er),
        .rx_clk_enable(pcs_sgmii_clk_en),
        .tx_clk_enable(pcs_sgmii_clk_en),
        .rx_mii_select(1'b0),
        .tx_mii_select(1'b0),
        .tx_error_underflow(),
        .tx_fifo_overflow(),
        .tx_fifo_bad_frame(),
        .tx_fifo_good_frame(),
        .rx_error_bad_frame(),
        .rx_error_bad_fcs(),
        .rx_fifo_overflow(),
        .rx_fifo_bad_frame(),
        .rx_fifo_good_frame(),
        .cfg_ifg(8'd12),
        .cfg_tx_enable(1'b1),
        .cfg_rx_enable(1'b1)
    );

    axis_async_fifo #(
        .DEPTH(4096),
        .DATA_WIDTH(8),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH(1),
        .LAST_ENABLE(1),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(1),
        .USER_WIDTH(1),
        .FRAME_FIFO(1),
        .DROP_OVERSIZE_FRAME(1),
        .DROP_BAD_FRAME(1)
    ) u_rx_axis_cdc (
        .s_clk(eth_clk_125),
        .s_rst(pcs_gmii_rst),
        .s_axis_tdata(rx_axis_mac_tdata),
        .s_axis_tkeep(rx_axis_mac_tkeep),
        .s_axis_tvalid(rx_axis_mac_tvalid),
        .s_axis_tready(rx_axis_mac_tready),
        .s_axis_tlast(rx_axis_mac_tlast),
        .s_axis_tid(8'd0),
        .s_axis_tdest(8'd0),
        .s_axis_tuser(rx_axis_mac_tuser),
        .m_clk(app_clk_100),
        .m_rst(app_rst),
        .m_axis_tdata(rx_axis_app_tdata),
        .m_axis_tkeep(rx_axis_app_tkeep),
        .m_axis_tvalid(rx_axis_app_tvalid),
        .m_axis_tready(rx_axis_app_tready),
        .m_axis_tlast(rx_axis_app_tlast),
        .m_axis_tid(),
        .m_axis_tdest(),
        .m_axis_tuser(rx_axis_app_tuser),
        .s_pause_req(1'b0),
        .s_pause_ack(),
        .m_pause_req(1'b0),
        .m_pause_ack(),
        .s_status_depth(),
        .s_status_depth_commit(),
        .s_status_overflow(),
        .s_status_bad_frame(),
        .s_status_good_frame(),
        .m_status_depth(),
        .m_status_depth_commit(),
        .m_status_overflow(),
        .m_status_bad_frame(),
        .m_status_good_frame()
    );

    axis_async_fifo #(
        .DEPTH(4096),
        .DATA_WIDTH(8),
        .KEEP_ENABLE(1),
        .KEEP_WIDTH(1),
        .LAST_ENABLE(1),
        .ID_ENABLE(0),
        .DEST_ENABLE(0),
        .USER_ENABLE(1),
        .USER_WIDTH(1),
        .FRAME_FIFO(1),
        .DROP_OVERSIZE_FRAME(1),
        .DROP_BAD_FRAME(1)
    ) u_tx_axis_cdc (
        .s_clk(app_clk_100),
        .s_rst(app_rst),
        .s_axis_tdata(tx_axis_app_tdata),
        .s_axis_tkeep(tx_axis_app_tkeep),
        .s_axis_tvalid(tx_axis_app_tvalid),
        .s_axis_tready(tx_axis_app_tready),
        .s_axis_tlast(tx_axis_app_tlast),
        .s_axis_tid(8'd0),
        .s_axis_tdest(8'd0),
        .s_axis_tuser(tx_axis_app_tuser),
        .m_clk(eth_clk_125),
        .m_rst(pcs_gmii_rst),
        .m_axis_tdata(tx_axis_mac_tdata),
        .m_axis_tkeep(tx_axis_mac_tkeep),
        .m_axis_tvalid(tx_axis_mac_tvalid),
        .m_axis_tready(tx_axis_mac_tready),
        .m_axis_tlast(tx_axis_mac_tlast),
        .m_axis_tid(),
        .m_axis_tdest(),
        .m_axis_tuser(tx_axis_mac_tuser),
        .s_pause_req(1'b0),
        .s_pause_ack(),
        .m_pause_req(1'b0),
        .m_pause_ack(),
        .s_status_depth(),
        .s_status_depth_commit(),
        .s_status_overflow(),
        .s_status_bad_frame(),
        .s_status_good_frame(),
        .m_status_depth(),
        .m_status_depth_commit(),
        .m_status_overflow(),
        .m_status_bad_frame(),
        .m_status_good_frame()
    );

    wire        rx_eth_hdr_valid;
    wire        rx_eth_hdr_ready;
    wire [47:0] rx_eth_dest_mac;
    wire [47:0] rx_eth_src_mac;
    wire [15:0] rx_eth_type;
    wire [7:0]  rx_eth_payload_axis_tdata;
    wire        rx_eth_payload_axis_tvalid;
    wire        rx_eth_payload_axis_tready;
    wire        rx_eth_payload_axis_tlast;
    wire        rx_eth_payload_axis_tuser;
    wire        rx_eth_payload_axis_tkeep;

    wire        tx_eth_hdr_valid;
    wire        tx_eth_hdr_ready;
    wire [47:0] tx_eth_dest_mac;
    wire [47:0] tx_eth_src_mac;
    wire [15:0] tx_eth_type;
    wire [7:0]  tx_eth_payload_axis_tdata;
    wire        tx_eth_payload_axis_tvalid;
    wire        tx_eth_payload_axis_tready;
    wire        tx_eth_payload_axis_tlast;
    wire        tx_eth_payload_axis_tuser;
    wire        tx_eth_payload_axis_tkeep = 1'b1;

    eth_axis_rx u_eth_axis_rx (
        .clk(app_clk_100),
        .rst(app_rst),
        .s_axis_tdata(rx_axis_app_tdata),
        .s_axis_tkeep(rx_axis_app_tkeep),
        .s_axis_tvalid(rx_axis_app_tvalid),
        .s_axis_tready(rx_axis_app_tready),
        .s_axis_tlast(rx_axis_app_tlast),
        .s_axis_tuser(rx_axis_app_tuser),
        .m_eth_hdr_valid(rx_eth_hdr_valid),
        .m_eth_hdr_ready(rx_eth_hdr_ready),
        .m_eth_dest_mac(rx_eth_dest_mac),
        .m_eth_src_mac(rx_eth_src_mac),
        .m_eth_type(rx_eth_type),
        .m_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
        .m_eth_payload_axis_tkeep(rx_eth_payload_axis_tkeep),
        .m_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
        .m_eth_payload_axis_tready(rx_eth_payload_axis_tready),
        .m_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
        .m_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
        .busy(),
        .error_header_early_termination()
    );

    eth_axis_tx u_eth_axis_tx (
        .clk(app_clk_100),
        .rst(app_rst),
        .s_eth_hdr_valid(tx_eth_hdr_valid),
        .s_eth_hdr_ready(tx_eth_hdr_ready),
        .s_eth_dest_mac(tx_eth_dest_mac),
        .s_eth_src_mac(tx_eth_src_mac),
        .s_eth_type(tx_eth_type),
        .s_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
        .s_eth_payload_axis_tkeep(tx_eth_payload_axis_tkeep),
        .s_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
        .s_eth_payload_axis_tready(tx_eth_payload_axis_tready),
        .s_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
        .s_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
        .m_axis_tdata(tx_axis_app_tdata),
        .m_axis_tkeep(tx_axis_app_tkeep),
        .m_axis_tvalid(tx_axis_app_tvalid),
        .m_axis_tready(tx_axis_app_tready),
        .m_axis_tlast(tx_axis_app_tlast),
        .m_axis_tuser(tx_axis_app_tuser),
        .busy()
    );

    wire        rx_udp_hdr_valid;
    wire        rx_udp_hdr_ready;
    wire [31:0] rx_udp_ip_source_ip;
    wire [15:0] rx_udp_source_port;
    wire [15:0] rx_udp_dest_port;
    wire [15:0] rx_udp_length;
    wire [7:0]  rx_udp_payload_axis_tdata;
    wire        rx_udp_payload_axis_tvalid;
    wire        rx_udp_payload_axis_tready;
    wire        rx_udp_payload_axis_tlast;
    wire        rx_udp_payload_axis_tuser;

    wire        tx_udp_hdr_valid;
    wire        tx_udp_hdr_ready;
    wire [5:0]  tx_udp_ip_dscp;
    wire [1:0]  tx_udp_ip_ecn;
    wire [7:0]  tx_udp_ip_ttl;
    wire [31:0] tx_udp_ip_source_ip_unused;
    wire [31:0] tx_udp_ip_dest_ip;
    wire [15:0] tx_udp_source_port;
    wire [15:0] tx_udp_dest_port;
    wire [15:0] tx_udp_length;
    wire [15:0] tx_udp_checksum;
    wire [7:0]  tx_udp_payload_axis_tdata;
    wire        tx_udp_payload_axis_tvalid;
    wire        tx_udp_payload_axis_tready;
    wire        tx_udp_payload_axis_tlast;
    wire        tx_udp_payload_axis_tuser;

    udp_complete u_udp (
        .clk(app_clk_100),
        .rst(app_rst),
        .s_eth_hdr_valid(rx_eth_hdr_valid),
        .s_eth_hdr_ready(rx_eth_hdr_ready),
        .s_eth_dest_mac(rx_eth_dest_mac),
        .s_eth_src_mac(rx_eth_src_mac),
        .s_eth_type(rx_eth_type),
        .s_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
        .s_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
        .s_eth_payload_axis_tready(rx_eth_payload_axis_tready),
        .s_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
        .s_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
        .m_eth_hdr_valid(tx_eth_hdr_valid),
        .m_eth_hdr_ready(tx_eth_hdr_ready),
        .m_eth_dest_mac(tx_eth_dest_mac),
        .m_eth_src_mac(tx_eth_src_mac),
        .m_eth_type(tx_eth_type),
        .m_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
        .m_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
        .m_eth_payload_axis_tready(tx_eth_payload_axis_tready),
        .m_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
        .m_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
        .s_ip_hdr_valid(1'b0),
        .s_ip_hdr_ready(),
        .s_ip_dscp(6'd0),
        .s_ip_ecn(2'd0),
        .s_ip_length(16'd0),
        .s_ip_ttl(8'd0),
        .s_ip_protocol(8'd0),
        .s_ip_source_ip(32'd0),
        .s_ip_dest_ip(32'd0),
        .s_ip_payload_axis_tdata(8'd0),
        .s_ip_payload_axis_tvalid(1'b0),
        .s_ip_payload_axis_tready(),
        .s_ip_payload_axis_tlast(1'b0),
        .s_ip_payload_axis_tuser(1'b0),
        .m_ip_hdr_valid(),
        .m_ip_hdr_ready(1'b1),
        .m_ip_eth_dest_mac(),
        .m_ip_eth_src_mac(),
        .m_ip_eth_type(),
        .m_ip_version(),
        .m_ip_ihl(),
        .m_ip_dscp(),
        .m_ip_ecn(),
        .m_ip_length(),
        .m_ip_identification(),
        .m_ip_flags(),
        .m_ip_fragment_offset(),
        .m_ip_ttl(),
        .m_ip_protocol(),
        .m_ip_header_checksum(),
        .m_ip_source_ip(),
        .m_ip_dest_ip(),
        .m_ip_payload_axis_tdata(),
        .m_ip_payload_axis_tvalid(),
        .m_ip_payload_axis_tready(1'b1),
        .m_ip_payload_axis_tlast(),
        .m_ip_payload_axis_tuser(),
        .s_udp_hdr_valid(tx_udp_hdr_valid),
        .s_udp_hdr_ready(tx_udp_hdr_ready),
        .s_udp_ip_dscp(tx_udp_ip_dscp),
        .s_udp_ip_ecn(tx_udp_ip_ecn),
        .s_udp_ip_ttl(tx_udp_ip_ttl),
        .s_udp_ip_source_ip(LOCAL_IP),
        .s_udp_ip_dest_ip(tx_udp_ip_dest_ip),
        .s_udp_source_port(tx_udp_source_port),
        .s_udp_dest_port(tx_udp_dest_port),
        .s_udp_length(tx_udp_length),
        .s_udp_checksum(tx_udp_checksum),
        .s_udp_payload_axis_tdata(tx_udp_payload_axis_tdata),
        .s_udp_payload_axis_tvalid(tx_udp_payload_axis_tvalid),
        .s_udp_payload_axis_tready(tx_udp_payload_axis_tready),
        .s_udp_payload_axis_tlast(tx_udp_payload_axis_tlast),
        .s_udp_payload_axis_tuser(tx_udp_payload_axis_tuser),
        .m_udp_hdr_valid(rx_udp_hdr_valid),
        .m_udp_hdr_ready(rx_udp_hdr_ready),
        .m_udp_eth_dest_mac(),
        .m_udp_eth_src_mac(),
        .m_udp_eth_type(),
        .m_udp_ip_version(),
        .m_udp_ip_ihl(),
        .m_udp_ip_dscp(),
        .m_udp_ip_ecn(),
        .m_udp_ip_length(),
        .m_udp_ip_identification(),
        .m_udp_ip_flags(),
        .m_udp_ip_fragment_offset(),
        .m_udp_ip_ttl(),
        .m_udp_ip_protocol(),
        .m_udp_ip_header_checksum(),
        .m_udp_ip_source_ip(rx_udp_ip_source_ip),
        .m_udp_ip_dest_ip(),
        .m_udp_source_port(rx_udp_source_port),
        .m_udp_dest_port(rx_udp_dest_port),
        .m_udp_length(rx_udp_length),
        .m_udp_checksum(),
        .m_udp_payload_axis_tdata(rx_udp_payload_axis_tdata),
        .m_udp_payload_axis_tvalid(rx_udp_payload_axis_tvalid),
        .m_udp_payload_axis_tready(rx_udp_payload_axis_tready),
        .m_udp_payload_axis_tlast(rx_udp_payload_axis_tlast),
        .m_udp_payload_axis_tuser(rx_udp_payload_axis_tuser),
        .ip_rx_busy(),
        .ip_tx_busy(),
        .udp_rx_busy(),
        .udp_tx_busy(),
        .ip_rx_error_header_early_termination(),
        .ip_rx_error_payload_early_termination(),
        .ip_rx_error_invalid_header(),
        .ip_rx_error_invalid_checksum(),
        .ip_tx_error_payload_early_termination(),
        .ip_tx_error_arp_failed(),
        .udp_rx_error_header_early_termination(),
        .udp_rx_error_payload_early_termination(),
        .udp_tx_error_payload_early_termination(),
        .local_mac(LOCAL_MAC),
        .local_ip(LOCAL_IP),
        .gateway_ip(GATEWAY_IP),
        .subnet_mask(SUBNET_MASK),
        .clear_arp_cache(1'b0)
    );

    //========================================================================
    //  통신 백엔드 + NPU 직결부  (DDR3/MIG/MicroBlaze/SmartConnect 전부 제거)
    //   PC --UDP--> oss_udp_bram_endpoint --32bit cmd--> npu_eth_backend
    //       --> npu_top (i_ld_*/i_start/m_axis_bbox)
    //========================================================================

    // ---- endpoint <-> backend 32-bit 명령 포트 ----
    wire [31:0] cmd_addr;
    wire [31:0] cmd_wdata;
    wire [31:0] cmd_rdata;
    wire        cmd_en;
    wire [3:0]  cmd_we;

    // ---- endpoint 상태(LED/디버그용, 미사용은 open) ----
    wire        endpoint_write_done_pulse;
    wire        endpoint_read_done_pulse;
    wire        endpoint_error_pulse;
    wire        endpoint_busy;
    wire [31:0] endpoint_last_seq;
    wire [15:0] endpoint_words_written;
    wire [15:0] endpoint_words_read;
    wire [7:0]  endpoint_status_code;

    oss_udp_bram_endpoint #(
        .UDP_PORT(16'd5005),
        .PAYLOAD_BYTES(1024),
        .BRAM_WORDS(BRAM_WORDS)
    ) u_endpoint (
        .clk(app_clk_100),
        .rst(app_rst),
        .enable(1'b1),
        .rx_udp_hdr_valid(rx_udp_hdr_valid),
        .rx_udp_hdr_ready(rx_udp_hdr_ready),
        .rx_udp_ip_source_ip(rx_udp_ip_source_ip),
        .rx_udp_source_port(rx_udp_source_port),
        .rx_udp_dest_port(rx_udp_dest_port),
        .rx_udp_length(rx_udp_length),
        .rx_udp_payload_axis_tdata(rx_udp_payload_axis_tdata),
        .rx_udp_payload_axis_tvalid(rx_udp_payload_axis_tvalid),
        .rx_udp_payload_axis_tready(rx_udp_payload_axis_tready),
        .rx_udp_payload_axis_tlast(rx_udp_payload_axis_tlast),
        .rx_udp_payload_axis_tuser(rx_udp_payload_axis_tuser),
        .tx_udp_hdr_valid(tx_udp_hdr_valid),
        .tx_udp_hdr_ready(tx_udp_hdr_ready),
        .tx_udp_ip_dscp(tx_udp_ip_dscp),
        .tx_udp_ip_ecn(tx_udp_ip_ecn),
        .tx_udp_ip_ttl(tx_udp_ip_ttl),
        .tx_udp_ip_source_ip(),                 // endpoint 내부 0 고정(미사용)
        .tx_udp_ip_dest_ip(tx_udp_ip_dest_ip),
        .tx_udp_source_port(tx_udp_source_port),
        .tx_udp_dest_port(tx_udp_dest_port),
        .tx_udp_length(tx_udp_length),
        .tx_udp_checksum(tx_udp_checksum),
        .tx_udp_payload_axis_tdata(tx_udp_payload_axis_tdata),
        .tx_udp_payload_axis_tvalid(tx_udp_payload_axis_tvalid),
        .tx_udp_payload_axis_tready(tx_udp_payload_axis_tready),
        .tx_udp_payload_axis_tlast(tx_udp_payload_axis_tlast),
        .tx_udp_payload_axis_tuser(tx_udp_payload_axis_tuser),
        // ---- 32-bit BRAM 포트 -> backend ----
        .bram_addr(cmd_addr),
        .bram_din(cmd_wdata),
        .bram_dout(cmd_rdata),
        .bram_en(cmd_en),
        .bram_we(cmd_we),
        .pc_write_done_pulse(endpoint_write_done_pulse),
        .pc_read_done_pulse(endpoint_read_done_pulse),
        .error_pulse(endpoint_error_pulse),
        .busy(endpoint_busy),
        .last_seq(endpoint_last_seq),
        .words_written(endpoint_words_written),
        .words_read(endpoint_words_read),
        .status_code(endpoint_status_code)
    );

    // ---- NPU <-> backend ----
    wire                npu_load_en;
    wire [13:0]         npu_ld_waddr;
    wire                npu_ld_we;
    wire [2047:0]       npu_ld_wdata;
    wire                npu_start;
    wire                npu_all_done;
    wire [95:0]         npu_bbox_tdata;
    wire                npu_bbox_tvalid;
    wire                npu_bbox_tready;

    npu_eth_backend #(
        .GFB_WIDTH(2048),
        .ADDR_WIDTH(14),
        .MAX_BOXES(MAX_BOXES)
    ) u_backend (
        .clk(app_clk_100),
        .rst(app_rst),
        .i_cmd_addr(cmd_addr),
        .i_cmd_wdata(cmd_wdata),
        .o_cmd_rdata(cmd_rdata),
        .i_cmd_en(cmd_en),
        .i_cmd_we(cmd_we),
        .o_load_en(npu_load_en),
        .o_ld_waddr(npu_ld_waddr),
        .o_ld_we(npu_ld_we),
        .o_ld_wdata(npu_ld_wdata),
        .o_start(npu_start),
        .i_all_done(npu_all_done),
        .i_bbox_tdata(npu_bbox_tdata),
        .i_bbox_tvalid(npu_bbox_tvalid),
        .o_bbox_tready(npu_bbox_tready),
        .o_busy(),
        .o_done(),
        .o_box_count()
    );

    npu_top #(
        .GFB_WIDTH(2048),
        .ADDR_WIDTH(14),
        .GFB_DEPTH(9600),
        .NUM_STEPS(57)
    ) u_npu (
        .clk(app_clk_100),
        .rst_n(~app_rst),
        .i_load_en(npu_load_en),
        .i_ld_waddr(npu_ld_waddr),
        .i_ld_we(npu_ld_we),
        .i_ld_wdata(npu_ld_wdata),
        .i_start(npu_start),
        .o_all_done(npu_all_done),
        .m_axis_tdata(npu_bbox_tdata),
        .m_axis_tvalid(npu_bbox_tvalid),
        .m_axis_tready(npu_bbox_tready)
    );

    // 미사용 endpoint 출력 경고 억제
    wire _unused_ep = &{1'b0, endpoint_write_done_pulse, endpoint_read_done_pulse,
                        endpoint_error_pulse, endpoint_busy, endpoint_last_seq,
                        endpoint_words_written, endpoint_words_read,
                        endpoint_status_code, 1'b0};

endmodule
