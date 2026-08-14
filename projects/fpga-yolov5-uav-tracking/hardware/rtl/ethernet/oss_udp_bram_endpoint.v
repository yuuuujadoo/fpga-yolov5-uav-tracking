`timescale 1 ns / 1 ps

module oss_udp_bram_endpoint #(
    parameter [15:0] UDP_PORT = 16'd5005,
    parameter integer PAYLOAD_BYTES = 1024,
    parameter integer BRAM_WORDS = 4096
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,

    input  wire        rx_udp_hdr_valid,
    output reg         rx_udp_hdr_ready,
    input  wire [31:0] rx_udp_ip_source_ip,
    input  wire [15:0] rx_udp_source_port,
    input  wire [15:0] rx_udp_dest_port,
    input  wire [15:0] rx_udp_length,
    input  wire [7:0]  rx_udp_payload_axis_tdata,
    input  wire        rx_udp_payload_axis_tvalid,
    output reg         rx_udp_payload_axis_tready,
    input  wire        rx_udp_payload_axis_tlast,
    input  wire        rx_udp_payload_axis_tuser,

    output reg         tx_udp_hdr_valid,
    input  wire        tx_udp_hdr_ready,
    output wire [5:0]  tx_udp_ip_dscp,
    output wire [1:0]  tx_udp_ip_ecn,
    output wire [7:0]  tx_udp_ip_ttl,
    output wire [31:0] tx_udp_ip_source_ip,
    output reg  [31:0] tx_udp_ip_dest_ip,
    output reg  [15:0] tx_udp_source_port,
    output reg  [15:0] tx_udp_dest_port,
    output wire [15:0] tx_udp_length,
    output wire [15:0] tx_udp_checksum,
    output reg  [7:0]  tx_udp_payload_axis_tdata,
    output reg         tx_udp_payload_axis_tvalid,
    input  wire        tx_udp_payload_axis_tready,
    output reg         tx_udp_payload_axis_tlast,
    output wire        tx_udp_payload_axis_tuser,

    output reg  [31:0] bram_addr,
    output reg  [31:0] bram_din,
    input  wire [31:0] bram_dout,
    output reg         bram_en,
    output reg  [3:0]  bram_we,

    output reg         pc_write_done_pulse,
    output reg         pc_read_done_pulse,
    output reg         error_pulse,
    output reg         busy,
    output reg  [31:0] last_seq,
    output reg  [15:0] words_written,
    output reg  [15:0] words_read,
    output reg  [7:0]  status_code
);

    localparam [31:0] MAGIC_LEGACY_WRITE = 32'h4d42524d; // "MBRM"
    localparam [31:0] MAGIC_WRITE        = 32'h4d425752; // "MBWR"
    localparam [31:0] MAGIC_READ         = 32'h4d425244; // "MBRD"

    localparam [7:0] STATUS_OK         = 8'd0;
    localparam [7:0] STATUS_BAD_MAGIC  = 8'd1;
    localparam [7:0] STATUS_BAD_LENGTH = 8'd2;

    localparam [3:0]
        ST_IDLE        = 4'd0,
        ST_DROP        = 4'd1,
        ST_RX          = 4'd2,
        ST_TX_HDR      = 4'd3,
        ST_TX_ACK      = 4'd4,
        ST_TX_RD_HDR   = 4'd5,
        ST_RD_REQ      = 4'd6,
        ST_RD_WAIT     = 4'd7,
        ST_RD_CAPTURE  = 4'd8,
        ST_TX_RD_WORD  = 4'd9;

    localparam [15:0] MAX_WORDS = PAYLOAD_BYTES/4;

    reg [3:0] state = ST_IDLE;
    reg [15:0] byte_count = 16'd0;
    reg [1:0] word_lane = 2'd0;
    reg [31:0] word_buf = 32'd0;
    reg [15:0] tx_index = 16'd0;
    reg [15:0] tx_payload_len = 16'd12;
    reg [31:0] req_addr = 32'd0;
    reg [15:0] req_words = MAX_WORDS;
    reg [31:0] magic = 32'd0;
    reg [31:0] tx_addr = 32'd0;
    reg [15:0] read_word_index = 16'd0;
    reg [1:0] read_byte_lane = 2'd0;
    reg [31:0] read_word = 32'd0;
    reg [7:0] tx_status_code = STATUS_OK;
    reg [7:0] tx_opcode = "W";

    assign tx_udp_ip_dscp = 6'd0;
    assign tx_udp_ip_ecn = 2'd0;
    assign tx_udp_ip_ttl = 8'd64;
    assign tx_udp_ip_source_ip = 32'd0;
    assign tx_udp_length = 16'd8 + tx_payload_len;
    assign tx_udp_checksum = 16'd0;
    assign tx_udp_payload_axis_tuser = 1'b0;

    wire unused_rx_user = rx_udp_payload_axis_tuser;
    wire [15:0] unused_rx_len = rx_udp_length;
    wire [31:0] unused_bram_words = BRAM_WORDS;

    function [7:0] ack_byte;
        input [7:0] i;
        begin
            case (i)
                8'd0: ack_byte = "A";
                8'd1: ack_byte = "C";
                8'd2: ack_byte = "K";
                8'd3: ack_byte = "!";
                8'd4: ack_byte = last_seq[31:24];
                8'd5: ack_byte = last_seq[23:16];
                8'd6: ack_byte = last_seq[15:8];
                8'd7: ack_byte = last_seq[7:0];
                8'd8: ack_byte = tx_opcode;
                8'd9: ack_byte = status_code;
                8'd10: ack_byte = words_written[15:8];
                8'd11: ack_byte = words_written[7:0];
                default: ack_byte = 8'd0;
            endcase
        end
    endfunction

    function [7:0] read_header_byte;
        input [7:0] i;
        begin
            case (i)
                8'd0: read_header_byte = "M";
                8'd1: read_header_byte = "B";
                8'd2: read_header_byte = "D";
                8'd3: read_header_byte = "T";
                8'd4: read_header_byte = last_seq[31:24];
                8'd5: read_header_byte = last_seq[23:16];
                8'd6: read_header_byte = last_seq[15:8];
                8'd7: read_header_byte = last_seq[7:0];
                8'd8: read_header_byte = tx_addr[31:24];
                8'd9: read_header_byte = tx_addr[23:16];
                8'd10: read_header_byte = tx_addr[15:8];
                8'd11: read_header_byte = tx_addr[7:0];
                8'd12: read_header_byte = words_read[15:8];
                8'd13: read_header_byte = words_read[7:0];
                8'd14: read_header_byte = status_code;
                8'd15: read_header_byte = 8'd0;
                default: read_header_byte = 8'd0;
            endcase
        end
    endfunction

    function [7:0] read_word_byte;
        input [31:0] word;
        input [1:0] lane;
        begin
            case (lane)
                2'd0: read_word_byte = word[7:0];
                2'd1: read_word_byte = word[15:8];
                2'd2: read_word_byte = word[23:16];
                default: read_word_byte = word[31:24];
            endcase
        end
    endfunction

    wire cmd_is_legacy_write = (magic == MAGIC_LEGACY_WRITE);
    wire cmd_is_write = (magic == MAGIC_WRITE);
    wire cmd_is_read = (magic == MAGIC_READ);
    wire rx_write_payload = (cmd_is_legacy_write && byte_count >= 16'd8) ||
                            (cmd_is_write && byte_count >= 16'd16);
    wire rx_final_write_word = rx_write_payload && (word_lane == 2'd3);
    wire [15:0] final_write_words = words_written + (rx_final_write_word ? 16'd1 : 16'd0);
    wire req_addr_aligned = (req_addr[1:0] == 2'b00);
    wire legacy_write_ok = cmd_is_legacy_write && (final_write_words == MAX_WORDS);
    wire extended_write_ok = cmd_is_write && (req_words <= MAX_WORDS) &&
                             (req_words != 16'd0) && req_addr_aligned &&
                             (final_write_words == req_words);
    wire read_ok = cmd_is_read && (req_words <= MAX_WORDS) &&
                   (req_words != 16'd0) && req_addr_aligned &&
                   (byte_count >= 16'd15);

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            rx_udp_hdr_ready <= 1'b0;
            rx_udp_payload_axis_tready <= 1'b0;
            tx_udp_hdr_valid <= 1'b0;
            tx_udp_payload_axis_tvalid <= 1'b0;
            tx_udp_payload_axis_tlast <= 1'b0;
            bram_en <= 1'b0;
            bram_we <= 4'd0;
            pc_write_done_pulse <= 1'b0;
            pc_read_done_pulse <= 1'b0;
            error_pulse <= 1'b0;
            busy <= 1'b0;
            last_seq <= 32'd0;
            words_written <= 16'd0;
            words_read <= 16'd0;
            status_code <= STATUS_OK;
            byte_count <= 16'd0;
            word_lane <= 2'd0;
            word_buf <= 32'd0;
            tx_index <= 16'd0;
            tx_payload_len <= 16'd12;
            req_addr <= 32'd0;
            req_words <= MAX_WORDS;
            magic <= 32'd0;
            tx_addr <= 32'd0;
            read_word_index <= 16'd0;
            read_byte_lane <= 2'd0;
            read_word <= 32'd0;
            tx_status_code <= STATUS_OK;
            tx_opcode <= "W";
            bram_addr <= 32'd0;
            bram_din <= 32'd0;
            tx_udp_ip_dest_ip <= 32'd0;
            tx_udp_source_port <= 16'd0;
            tx_udp_dest_port <= 16'd0;
            tx_udp_payload_axis_tdata <= 8'd0;
        end else begin
            rx_udp_hdr_ready <= 1'b0;
            rx_udp_payload_axis_tready <= 1'b0;
            bram_en <= 1'b0;
            bram_we <= 4'd0;
            pc_write_done_pulse <= 1'b0;
            pc_read_done_pulse <= 1'b0;
            error_pulse <= 1'b0;
            busy <= (state != ST_IDLE);

            case (state)
                ST_IDLE: begin
                    rx_udp_hdr_ready <= 1'b1;
                    if (rx_udp_hdr_valid && rx_udp_hdr_ready) begin
                        tx_udp_ip_dest_ip <= rx_udp_ip_source_ip;
                        tx_udp_source_port <= UDP_PORT;
                        tx_udp_dest_port <= rx_udp_source_port;
                        byte_count <= 16'd0;
                        word_lane <= 2'd0;
                        words_written <= 16'd0;
                        words_read <= 16'd0;
                        status_code <= STATUS_OK;
                        tx_status_code <= STATUS_OK;
                        magic <= 32'd0;
                        req_addr <= 32'd0;
                        req_words <= MAX_WORDS;
                        tx_addr <= 32'd0;
                        if (rx_udp_dest_port == UDP_PORT && enable) begin
                            state <= ST_RX;
                        end else begin
                            state <= ST_DROP;
                        end
                    end
                end

                ST_DROP: begin
                    rx_udp_payload_axis_tready <= 1'b1;
                    if (rx_udp_payload_axis_tvalid && rx_udp_payload_axis_tready &&
                        rx_udp_payload_axis_tlast) begin
                        state <= ST_IDLE;
                    end
                end

                ST_RX: begin
                    rx_udp_payload_axis_tready <= 1'b1;
                    if (rx_udp_payload_axis_tvalid && rx_udp_payload_axis_tready) begin
                        if (byte_count < 16'd4) begin
                            magic <= {magic[23:0], rx_udp_payload_axis_tdata};
                        end
                        if (byte_count >= 16'd4 && byte_count < 16'd8) begin
                            last_seq <= {last_seq[23:0], rx_udp_payload_axis_tdata};
                        end
                        if ((cmd_is_write || cmd_is_read) && byte_count >= 16'd8 && byte_count < 16'd12) begin
                            req_addr <= {req_addr[23:0], rx_udp_payload_axis_tdata};
                        end
                        if ((cmd_is_write || cmd_is_read) && byte_count >= 16'd12 && byte_count < 16'd14) begin
                            req_words <= {req_words[7:0], rx_udp_payload_axis_tdata};
                        end

                        if (rx_write_payload) begin
                            case (word_lane)
                                2'd0: word_buf[7:0] <= rx_udp_payload_axis_tdata;
                                2'd1: word_buf[15:8] <= rx_udp_payload_axis_tdata;
                                2'd2: word_buf[23:16] <= rx_udp_payload_axis_tdata;
                                default: begin
                                    bram_addr <= req_addr + {14'd0, words_written, 2'b00};
                                    bram_din <= {rx_udp_payload_axis_tdata, word_buf[23:0]};
                                    bram_en <= 1'b1;
                                    bram_we <= 4'hf;
                                    words_written <= words_written + 16'd1;
                                end
                            endcase
                            word_lane <= word_lane + 2'd1;
                        end

                        byte_count <= byte_count + 16'd1;

                        if (rx_udp_payload_axis_tlast) begin
                            if (legacy_write_ok || extended_write_ok) begin
                                status_code <= STATUS_OK;
                                tx_status_code <= STATUS_OK;
                                tx_opcode <= "W";
                                tx_payload_len <= 16'd12;
                                words_written <= final_write_words;
                                pc_write_done_pulse <= 1'b1;
                                tx_udp_hdr_valid <= 1'b1;
                                state <= ST_TX_HDR;
                            end else if (read_ok) begin
                                status_code <= STATUS_OK;
                                tx_status_code <= STATUS_OK;
                                tx_opcode <= "R";
                                words_read <= req_words;
                                tx_addr <= req_addr;
                                read_word_index <= 16'd0;
                                tx_payload_len <= 16'd16 + (req_words << 2);
                                tx_udp_hdr_valid <= 1'b1;
                                state <= ST_TX_HDR;
                            end else begin
                                if (cmd_is_legacy_write || cmd_is_write || cmd_is_read) begin
                                    status_code <= STATUS_BAD_LENGTH;
                                    tx_status_code <= STATUS_BAD_LENGTH;
                                end else begin
                                    status_code <= STATUS_BAD_MAGIC;
                                    tx_status_code <= STATUS_BAD_MAGIC;
                                end
                                tx_opcode <= cmd_is_read ? "R" : "W";
                                words_written <= final_write_words;
                                words_read <= 16'd0;
                                tx_payload_len <= 16'd12;
                                error_pulse <= 1'b1;
                                tx_udp_hdr_valid <= 1'b1;
                                state <= ST_TX_HDR;
                            end
                        end
                    end
                end

                ST_TX_HDR: begin
                    if (tx_udp_hdr_valid && tx_udp_hdr_ready) begin
                        tx_udp_hdr_valid <= 1'b0;
                        tx_index <= 16'd0;
                        if (tx_opcode == "R" && tx_status_code == STATUS_OK) begin
                            tx_udp_payload_axis_tdata <= read_header_byte(8'd0);
                            tx_udp_payload_axis_tvalid <= 1'b1;
                            tx_udp_payload_axis_tlast <= 1'b0;
                            state <= ST_TX_RD_HDR;
                        end else begin
                            tx_udp_payload_axis_tdata <= ack_byte(8'd0);
                            tx_udp_payload_axis_tvalid <= 1'b1;
                            tx_udp_payload_axis_tlast <= 1'b0;
                            state <= ST_TX_ACK;
                        end
                    end
                end

                ST_TX_ACK: begin
                    if (tx_udp_payload_axis_tvalid && tx_udp_payload_axis_tready) begin
                        if (tx_index == 16'd11) begin
                            tx_udp_payload_axis_tvalid <= 1'b0;
                            tx_udp_payload_axis_tlast <= 1'b0;
                            state <= ST_IDLE;
                        end else begin
                            tx_index <= tx_index + 16'd1;
                            tx_udp_payload_axis_tdata <= ack_byte(tx_index[7:0] + 8'd1);
                            tx_udp_payload_axis_tlast <= (tx_index == 16'd10);
                        end
                    end
                end

                ST_TX_RD_HDR: begin
                    if (tx_udp_payload_axis_tvalid && tx_udp_payload_axis_tready) begin
                        if (tx_index == 16'd15) begin
                            tx_udp_payload_axis_tvalid <= 1'b0;
                            tx_udp_payload_axis_tlast <= 1'b0;
                            if (words_read == 16'd0) begin
                                pc_read_done_pulse <= 1'b1;
                                state <= ST_IDLE;
                            end else begin
                                state <= ST_RD_REQ;
                            end
                        end else begin
                            tx_index <= tx_index + 16'd1;
                            tx_udp_payload_axis_tdata <= read_header_byte(tx_index[7:0] + 8'd1);
                            tx_udp_payload_axis_tlast <= (tx_index == 16'd14 && words_read == 16'd0);
                        end
                    end
                end

                ST_RD_REQ: begin
                    bram_addr <= tx_addr + {14'd0, read_word_index, 2'b00};
                    bram_en <= 1'b1;
                    bram_we <= 4'd0;
                    state <= ST_RD_WAIT;
                end

                ST_RD_WAIT: begin
                    state <= ST_RD_CAPTURE;
                end

                ST_RD_CAPTURE: begin
                    read_word <= bram_dout;
                    read_byte_lane <= 2'd0;
                    tx_udp_payload_axis_tdata <= bram_dout[7:0];
                    tx_udp_payload_axis_tvalid <= 1'b1;
                    tx_udp_payload_axis_tlast <= 1'b0;
                    state <= ST_TX_RD_WORD;
                end

                ST_TX_RD_WORD: begin
                    if (tx_udp_payload_axis_tvalid && tx_udp_payload_axis_tready) begin
                        if (read_byte_lane == 2'd3) begin
                            tx_udp_payload_axis_tvalid <= 1'b0;
                            tx_udp_payload_axis_tlast <= 1'b0;
                            if (read_word_index == words_read - 16'd1) begin
                                pc_read_done_pulse <= 1'b1;
                                state <= ST_IDLE;
                            end else begin
                                read_word_index <= read_word_index + 16'd1;
                                state <= ST_RD_REQ;
                            end
                        end else begin
                            read_byte_lane <= read_byte_lane + 2'd1;
                            tx_udp_payload_axis_tdata <= read_word_byte(read_word, read_byte_lane + 2'd1);
                            tx_udp_payload_axis_tlast <= (read_byte_lane == 2'd2 &&
                                                          read_word_index == words_read - 16'd1);
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
