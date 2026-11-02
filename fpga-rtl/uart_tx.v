`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// uart_tx : 8N1 transmitter, LSB first
//   BAUD_CNT = CLK_FREQ / BAUD  (50 MHz / 115200 = 434)
//   start is sampled while idle; busy stays high until the stop bit is done.
// -----------------------------------------------------------------------------
module uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data,
    output reg        busy,
    output reg        txd
);

    localparam integer BAUD_CNT = CLK_FREQ / BAUD;

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]  st;
    reg [15:0] bcnt;
    reg [2:0]  bit_i;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st    <= S_IDLE;
            bcnt  <= 16'd0;
            bit_i <= 3'd0;
            sh    <= 8'd0;
            busy  <= 1'b0;
            txd   <= 1'b1;
        end else begin
            case (st)
            S_IDLE: begin
                txd <= 1'b1;
                if (start) begin
                    sh    <= data;
                    bcnt  <= 16'd0;
                    bit_i <= 3'd0;
                    txd   <= 1'b0;      // start bit
                    busy  <= 1'b1;
                    st    <= S_START;
                end
            end
            S_START: begin
                if (bcnt == BAUD_CNT - 1) begin
                    bcnt <= 16'd0;
                    txd  <= sh[0];      // bit 0
                    sh   <= {1'b1, sh[7:1]};
                    st   <= S_DATA;
                end else begin
                    bcnt <= bcnt + 1'b1;
                end
            end
            S_DATA: begin
                if (bcnt == BAUD_CNT - 1) begin
                    bcnt <= 16'd0;
                    if (bit_i == 3'd7) begin
                        txd <= 1'b1;    // stop bit
                        st  <= S_STOP;
                    end else begin
                        bit_i <= bit_i + 1'b1;
                        txd   <= sh[0];
                        sh    <= {1'b1, sh[7:1]};
                    end
                end else begin
                    bcnt <= bcnt + 1'b1;
                end
            end
            S_STOP: begin
                if (bcnt == BAUD_CNT - 1) begin
                    bcnt <= 16'd0;
                    busy <= 1'b0;
                    st   <= S_IDLE;
                end else begin
                    bcnt <= bcnt + 1'b1;
                end
            end
            endcase
        end
    end

endmodule
