`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// osd_cross : draw two crosshair overlays on the LCD pixel stream
//
//   read side (lcd_clk domain). Centroids come from the camera domain and only
//   change once per frame, so a 2-FF synchroniser is enough.
//
//   each crosshair is drawn as a light core with a dark 1-px border, so it stays
//   visible on both the white paper and the dark red / blue targets.
//
//   xpos/ypos are the LCD pixel coordinates from lcd_rgb_top; rgb_in is the
//   frame-buffer pixel. rgb_out goes back into lcd_rgb_top's data_in.
// -----------------------------------------------------------------------------
module osd_cross #(
    parameter integer ARM        = 10,          // arm length in pixels
    parameter integer THICK      = 1,           // core half thickness (1 -> 3 px)
    parameter [15:0]  COL_MARKER = 16'hFFFF,    // white core  (marker)
    parameter [15:0]  COL_OBJECT = 16'h07E0,    // green core  (object)
    parameter [15:0]  COL_BORDER = 16'h0000,    // black border around both
    parameter integer H_PIXEL    = 800,         // for the x un-flip
    parameter integer X_FLIP     = 1            // centroids arrive x-mirrored
)(
    input  wire        clk,          // lcd_clk
    input  wire        rst_n,
    input  wire [10:0] xpos,
    input  wire [10:0] ypos,
    input  wire [15:0] rgb_in,
    input  wire [15:0] mx,
    input  wire [15:0] my,
    input  wire        m_found,
    input  wire [15:0] ox,
    input  wire [15:0] oy,
    input  wire        o_found,
    output reg  [15:0] rgb_out
);

    // ---------------- 2-FF synchroniser ----------------
    reg [15:0] mx1, mx2, my1, my2;
    reg [15:0] ox1, ox2, oy1, oy2;
    reg        mf1, mf2, of1, of2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mx1 <= 0; mx2 <= 0; my1 <= 0; my2 <= 0;
            ox1 <= 0; ox2 <= 0; oy1 <= 0; oy2 <= 0;
            mf1 <= 0; mf2 <= 0; of1 <= 0; of2 <= 0;
        end else begin
            mx1 <= mx; mx2 <= mx1;
            my1 <= my; my2 <= my1;
            ox1 <= ox; ox2 <= ox1;
            oy1 <= oy; oy2 <= oy1;
            mf1 <= m_found; mf2 <= mf1;
            of1 <= o_found; of2 <= of1;
        end
    end

    wire [11:0] xp = {1'b0, xpos};
    wire [11:0] yp = {1'b0, ypos};
    // centroids are reported with x mirrored; un-flip here so the crosshair
    // lands on the real object instead of its mirror image
    wire [11:0] mx2_12 = X_FLIP ? (H_PIXEL - 1 - mx2[11:0]) : mx2[11:0];
    wire [11:0] my2_12 = my2[11:0];
    wire [11:0] ox2_12 = X_FLIP ? (H_PIXEL - 1 - ox2[11:0]) : ox2[11:0];
    wire [11:0] oy2_12 = oy2[11:0];

    function hit;
        input [11:0] px;
        input [11:0] py;
        input [11:0] cx;
        input [11:0] cy;
        input integer thk;
        reg   [11:0] dx;
        reg   [11:0] dy;
        begin
            dx = (px > cx) ? (px - cx) : (cx - px);
            dy = (py > cy) ? (py - cy) : (cy - py);
            hit = ((dx <= thk) && (dy <= ARM)) ||
                  ((dy <= thk) && (dx <= ARM));
        end
    endfunction

    wire hit_m_core = mf2 && hit(xp, yp, mx2_12, my2_12, THICK);
    wire hit_m_edge = mf2 && hit(xp, yp, mx2_12, my2_12, THICK + 1);
    wire hit_o_core = of2 && hit(xp, yp, ox2_12, oy2_12, THICK);
    wire hit_o_edge = of2 && hit(xp, yp, ox2_12, oy2_12, THICK + 1);

    always @(*) begin
        if (hit_m_edge)      rgb_out = hit_m_core ? COL_MARKER : COL_BORDER;
        else if (hit_o_edge) rgb_out = hit_o_core ? COL_OBJECT : COL_BORDER;
        else                 rgb_out = rgb_in;
    end

endmodule
