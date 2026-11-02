`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// feat_uart : centroid results + servo commands -> ASCII line on UART
//
//   cam_pclk domain : latch {cx_r,cy_r,cx_b,cy_b, found, d_pan, d_reach}, toggle
//   clk_50m  domain : sync the toggle, latch, decimal-convert, send 35 bytes:
//
//     R236,211 B429,310 F11 P+004 E-003\r\n
//
//     R,B      : red (marker) / blue (object) centroids, 3 digits each
//     F        : found_red, found_blue
//     P,E      : pan / reach command in ticks, signed, 3 digits
//
//   UART 115200 8N1, one line per frame (~3 ms).
// -----------------------------------------------------------------------------
module feat_uart #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire        cam_pclk,
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        feat_done,
    input  wire [15:0] cx_r,
    input  wire [15:0] cy_r,
    input  wire [15:0] cx_b,
    input  wire [15:0] cy_b,
    input  wire        found_r,
    input  wire        found_b,
    input  wire signed [15:0] d_pan,
    input  wire signed [15:0] d_reach,
    output wire        uart_txd ,
    // 2026-10-28: second physical copy of the same stream, pinned to the
    // ATK MODULE header (D16) so an ESP32 can sniff it without tapping the CH340 net.
    output wire        uart_txd2
);

    localparam integer NBYTE = 35;

    // ------------------------------------------------------------------ CDC
    reg [95:0] pay;         // {cx_r, cy_r, cx_b, cy_b, d_pan, d_reach} = 6 x 16 bits
    reg [1:0]  flg;
    reg        tog;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            pay <= 96'd0;
            flg <= 2'd0;
            tog <= 1'b0;
        end else if (feat_done) begin
            pay <= {cx_r, cy_r, cx_b, cy_b, d_pan, d_reach};
            flg <= {found_r, found_b};
            tog <= ~tog;
        end
    end

    reg [2:0] tog_s;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) tog_s <= 3'b000;
        else        tog_s <= {tog_s[1:0], tog};
    end

    wire new_frame = tog_s[2] ^ tog_s[1];

    // ------------------------------------------------------------------ FSM
    localparam F_IDLE = 3'd0;
    localparam F_HUND = 3'd1;
    localparam F_TENS = 3'd2;
    localparam F_STOR = 3'd3;
    localparam F_SEND = 3'd4;

    reg [2:0]  fst;
    reg [2:0]  ch;          // 0..5
    reg [15:0] vxr, vyr, vxb, vyb;
    reg signed [15:0] vpan, vrch;
    reg [1:0]  vflg;
    reg        sgn_pan, sgn_rch;
    reg [15:0] rem;
    reg [3:0]  d100, d10;
    reg [3:0]  dg [0:17];
    reg [5:0]  idx;
    reg [7:0]  tx_byte;
    reg        tx_start;

    wire       tx_busy;

    // absolute values for the two signed commands
    wire [15:0] abs_pan = vpan[15] ? (16'd0 - vpan) : vpan;
    wire [15:0] abs_rch = vrch[15] ? (16'd0 - vrch) : vrch;

    // value selected by channel index (used to reload the converter)
    function [15:0] val_at;
        input [2:0] c;
        begin
            case (c)
            3'd0: val_at = vxr;
            3'd1: val_at = vyr;
            3'd2: val_at = vxb;
            3'd3: val_at = vyb;
            3'd4: val_at = abs_pan;
            3'd5: val_at = abs_rch;
            default: val_at = 16'd0;
            endcase
        end
    endfunction

    // value currently being converted
    reg [15:0] cur_val;
    always @(*) begin
        case (ch)
        3'd0: cur_val = vxr;
        3'd1: cur_val = vyr;
        3'd2: cur_val = vxb;
        3'd3: cur_val = vyb;
        3'd4: cur_val = abs_pan;
        default: cur_val = abs_rch;
        endcase
    end

    wire [3:0] dbase = (ch == 3'd0) ? 4'd0  :
                       (ch == 3'd1) ? 4'd3 :
                       (ch == 3'd2) ? 4'd6 :
                       (ch == 3'd3) ? 4'd9 :
                       (ch == 3'd4) ? 4'd12 : 4'd15;

    // byte 0..34 of one line
    function [7:0] fbyte;
        input [5:0] i;
        begin
            case (i)
            6'd0:  fbyte = "R";
            6'd1:  fbyte = {4'b0011, dg[0]};
            6'd2:  fbyte = {4'b0011, dg[1]};
            6'd3:  fbyte = {4'b0011, dg[2]};
            6'd4:  fbyte = ",";
            6'd5:  fbyte = {4'b0011, dg[3]};
            6'd6:  fbyte = {4'b0011, dg[4]};
            6'd7:  fbyte = {4'b0011, dg[5]};
            6'd8:  fbyte = " ";
            6'd9:  fbyte = "B";
            6'd10: fbyte = {4'b0011, dg[6]};
            6'd11: fbyte = {4'b0011, dg[7]};
            6'd12: fbyte = {4'b0011, dg[8]};
            6'd13: fbyte = ",";
            6'd14: fbyte = {4'b0011, dg[9]};
            6'd15: fbyte = {4'b0011, dg[10]};
            6'd16: fbyte = {4'b0011, dg[11]};
            6'd17: fbyte = " ";
            6'd18: fbyte = "F";
            6'd19: fbyte = {4'b0011, 3'b000, vflg[1]};
            6'd20: fbyte = {4'b0011, 3'b000, vflg[0]};
            6'd21: fbyte = " ";
            6'd22: fbyte = "P";
            6'd23: fbyte = sgn_pan ? "-" : "+";
            6'd24: fbyte = {4'b0011, dg[12]};
            6'd25: fbyte = {4'b0011, dg[13]};
            6'd26: fbyte = {4'b0011, dg[14]};
            6'd27: fbyte = " ";
            6'd28: fbyte = "E";
            6'd29: fbyte = sgn_rch ? "-" : "+";
            6'd30: fbyte = {4'b0011, dg[15]};
            6'd31: fbyte = {4'b0011, dg[16]};
            6'd32: fbyte = {4'b0011, dg[17]};
            6'd33: fbyte = 8'h0D;
            default: fbyte = 8'h0A;
            endcase
        end
    endfunction

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fst <= F_IDLE; ch <= 3'd0; rem <= 16'd0;
            d100 <= 4'd0; d10 <= 4'd0;
            vxr <= 0; vyr <= 0; vxb <= 0; vyb <= 0; vpan <= 0; vrch <= 0;
            vflg <= 2'd0; sgn_pan <= 1'b0; sgn_rch <= 1'b0;
            idx <= 6'd0; tx_byte <= 8'd0; tx_start <= 1'b0;
        end else begin
            case (fst)
            F_IDLE: begin
                tx_start <= 1'b0;
                if (new_frame) begin
                    vxr  <= pay[95:80];
                    vyr  <= pay[79:64];
                    vxb  <= pay[63:48];
                    vyb  <= pay[47:32];
                    vpan <= $signed(pay[31:16]);
                    vrch <= $signed(pay[15:0]);
                    vflg <= flg;
                    sgn_pan <= pay[31];
                    sgn_rch <= pay[15];
                    ch   <= 3'd0;
                    rem  <= pay[95:80];
                    d100 <= 4'd0;
                    d10  <= 4'd0;
                    fst  <= F_HUND;
                end
            end
            F_HUND: begin
                if (rem >= 16'd100) begin
                    rem  <= rem - 16'd100;
                    d100 <= d100 + 1'b1;
                end else begin
                    fst <= F_TENS;
                end
            end
            F_TENS: begin
                if (rem >= 16'd10) begin
                    rem <= rem - 16'd10;
                    d10 <= d10 + 1'b1;
                end else begin
                    fst <= F_STOR;
                end
            end
            F_STOR: begin
                dg[dbase]     <= d100;
                dg[dbase + 1] <= d10;
                dg[dbase + 2] <= rem[3:0];
                ch <= ch + 3'd1;
                if (ch == 3'd5) begin
                    idx <= 6'd0;
                    fst <= F_SEND;
                end else begin
                    d100 <= 4'd0;
                    d10  <= 4'd0;
                    rem  <= val_at(ch + 3'd1);   // reload for the next value
                    fst  <= F_HUND;
                end
            end
            F_SEND: begin
                tx_start <= 1'b0;
                if (!tx_busy && !tx_start) begin
                    tx_byte  <= fbyte(idx);
                    tx_start <= 1'b1;
                    if (idx == NBYTE - 1) begin
                        fst <= F_IDLE;
                    end else begin
                        idx <= idx + 6'd1;
                    end
                end
            end
            default: fst <= F_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------ UART

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_tx (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .start (tx_start),
        .data  (tx_byte),
        .busy  (tx_busy),
        .txd   (uart_txd)
    );

    // second instance: identical lockstep copy driving the header pin
    wire tx_busy2;   // busy of the 2nd instance (unused, kept for port alignment)

    uart_tx #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_tx2 (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .start (tx_start),
        .data  (tx_byte),
        .busy  (tx_busy2),
        .txd   (uart_txd2)
    );

endmodule
