`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// rgb_color_mask : RGB565 -> red / blue pixel masks (channel-dominance rule)
//   red  : R dominates G and B by MARGIN, and R is bright enough
//   blue : B dominates R and G by MARGIN, and B is bright enough
//   No HSV, no divider. 1-cycle latency; o_valid is delayed with the data.
// -----------------------------------------------------------------------------
module rgb_color_mask #(
    parameter [7:0] MARGIN   = 8'd40,   // dominance margin (tune from P0.2)
    parameter [7:0] MIN_RED  = 8'd90,   // reject dark / dirty red
    parameter [7:0] MIN_BLUE = 8'd70    // blue is darker in luma, lower gate
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_valid,
    input  wire [15:0] i_rgb,      // RGB565
    output reg         o_valid,
    output reg         o_red,
    output reg         o_blue
);

    // ---- unpack RGB565 to 8-bit channels ----
    wire [7:0] r = {i_rgb[15:11], 3'b000};
    wire [7:0] g = {i_rgb[10:5],  2'b00};
    wire [7:0] b = {i_rgb[4:0],   3'b000};

    // ---- 9-bit sums so "channel + margin" cannot overflow 8 bits ----
    wire [8:0] r9 = {1'b0, r};
    wire [8:0] g9 = {1'b0, g};
    wire [8:0] b9 = {1'b0, b};
    wire [8:0] r_m = r9 + {1'b0, MARGIN};
    wire [8:0] g_m = g9 + {1'b0, MARGIN};
    wire [8:0] b_m = b9 + {1'b0, MARGIN};

    // ---- dominance tests ----
    // red  : R must clearly beat both G and B
    // blue : B must clearly beat R, and B must be at least as strong as G
    //        (a sky-blue / cyan object has G close to B, so a strict "B > G+margin"
    //         rule rejects it; "B >= G" still rejects white, grey and green)
    wire red_pix  = (r9 > g_m) && (r9 > b_m) && (r > MIN_RED);
    wire blue_pix = (b9 > r_m) && (b9 >= g9) && (b > MIN_BLUE);

    // ---- registered outputs, valid delayed with the data ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_valid <= 1'b0;
            o_red   <= 1'b0;
            o_blue  <= 1'b0;
        end else begin
            o_valid <= i_valid;
            o_red   <= red_pix;
            o_blue  <= blue_pix;
        end
    end

endmodule
