`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// uart_rx : 8N1 receiver, 3-stage synchroniser, mid-bit sampling
//   50 MHz / 115200 -> BAUD_CNT = 434
//   o_valid is a 1-cycle pulse; o_data holds the byte
// -----------------------------------------------------------------------------
module uart_rx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,
    output reg        o_valid,
    output reg  [7:0] o_data
);

    localparam integer BAUD_CNT = CLK_FREQ / BAUD;

    // 3-stage synchroniser
    reg [2:0] rx_s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) rx_s <= 3'b111;
        else        rx_s <= {rx_s[1:0], rxd};

    wire rx_bit = rx_s[2];
    wire start  = rx_s[2] & ~rx_s[1];   // falling edge on the synchronised line

    localparam S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;

    reg [1:0]  st;
    reg [15:0] bcnt;
    reg [2:0]  bit_i;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; bcnt <= 16'd0; bit_i <= 3'd0; sh <= 8'd0;
            o_valid <= 1'b0; o_data <= 8'd0;
        end else begin
            o_valid <= 1'b0;
            case (st)
            S_IDLE: begin
                if (start) begin
                    bcnt <= BAUD_CNT / 2;    // move to the middle of the start bit
                    st   <= S_START;
                end
            end
            S_START: begin
                if (bcnt == 0) begin
                    bcnt  <= BAUD_CNT - 1;
                    bit_i <= 3'd0;
                    st    <= S_DATA;
                end else begin
                    bcnt <= bcnt - 1'b1;
                end
            end
            S_DATA: begin
                if (bcnt == 0) begin
                    sh[bit_i] <= rx_bit;
                    bcnt      <= BAUD_CNT - 1;
                    if (bit_i == 3'd7) st <= S_STOP;
                    else               bit_i <= bit_i + 1'b1;
                end else begin
                    bcnt <= bcnt - 1'b1;
                end
            end
            S_STOP: begin
                if (bcnt == 0) begin
                    o_valid <= 1'b1;
                    o_data  <= sh;
                    st      <= S_IDLE;
                end else begin
                    bcnt <= bcnt - 1'b1;
                end
            end
            endcase
        end
    end

endmodule
