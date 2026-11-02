`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// servo_polar : image-space polar visual servo
//
//   e      = s_object - s_marker                (pixels)
//   u_*    = s_* - c                            (c = projection of the base axis)
//   d_pan  = K_THETA * cross(u_marker, e)       angle channel
//   d_reach= K_R     * (|u_obj|^2 - |u_marker|^2)   radial channel
//
//   both channels are rate limited to +/-LIMIT ticks per frame and zeroed
//   inside DEADZONE. After HOLD_N consecutive frames inside the deadzone the
//   servo asserts o_hold and stops issuing increments.
//
//   units: one tick is whatever the ESP32 maps it to (e.g. 0.2 deg / 2 mm)
//   gains are Q10 (value = K_THETA_Q10 / 1024)
// -----------------------------------------------------------------------------
module servo_polar #(
    parameter integer C_X          = 400,   // pole (image projection of base axis)
    parameter integer C_Y          = 240,
    parameter integer K_THETA_Q10  = 1,     // angle gain, Q10
    parameter integer K_R_Q10      = 24,    // radial gain, Q10
    parameter integer DEADZONE     = 6,     // pixels
    parameter integer LIMIT        = 6,     // max ticks per frame
    parameter integer HOLD_N       = 5,     // frames inside deadzone to latch HOLD
    /* ★ 2026-10-28 实测补丁：输出标度（原来固定 >>10）。
     * cross 的量纲是 px^2（半径 ~400px × 角误差 px），典型 3~5 万；dr2 更大。
     * 原来 >>10 且 K_Q10=4 时输出 100+，**永远钳在 LIMIT**，闭环退化成 bang-bang，
     * 实测表现为"甩过头再甩回来"（半径 437->503->202）。
     * PAN=16 / RCH=17 时：20° 角误差 -> ~3.4 tick，200px 半径差 -> ~6.5 tick，
     * 只有大误差才饱和，中等误差区恢复成比例控制。 */
    parameter integer SHIFT_PAN   = 15,
    parameter integer SHIFT_RCH   = 16
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        feat_done,
    input  wire [15:0] cx_r, cy_r,
    input  wire [15:0] cx_b, cy_b,
    input  wire        found_r,
    input  wire        found_b,
    input  wire [15:0] cfg_cx,     // runtime pole (from cmd_parse)
    input  wire [15:0] cfg_cy,
    output reg  signed [15:0] d_pan,
    output reg  signed [15:0] d_reach,
    output reg         o_valid,
    output reg         o_hold
);

    // ---------------- signed intermediates ----------------
    wire signed [15:0] cxr = cx_r;
    wire signed [15:0] cyr = cy_r;
    wire signed [15:0] cxb = cx_b;
    wire signed [15:0] cyb = cy_b;

    wire signed [15:0] ex  = cxb - cxr;
    wire signed [15:0] ey  = cyb - cyr;

    // 2-FF sync of the runtime pole (it changes only when the PC sends a command)
    reg [15:0] cx_s1, cx_s2, cy_s1, cy_s2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cx_s1 <= 16'd0; cx_s2 <= 16'd0;
            cy_s1 <= 16'd0; cy_s2 <= 16'd0;
        end else begin
            cx_s1 <= cfg_cx; cx_s2 <= cx_s1;
            cy_s1 <= cfg_cy; cy_s2 <= cy_s1;
        end
    end

    wire signed [15:0] umx = cxr - $signed(cx_s2);
    wire signed [15:0] umy = cyr - $signed(cy_s2);
    wire signed [15:0] ubx = cxb - $signed(cx_s2);
    wire signed [15:0] uby = cyb - $signed(cy_s2);

    // cross product z component and squared radii
    wire signed [31:0] cross_z = umx * ey - umy * ex;
    wire signed [31:0] rm2     = umx * umx + umy * umy;
    wire signed [31:0] rb2     = ubx * ubx + uby * uby;
    wire signed [31:0] dr2     = rb2 - rm2;

    // squared error for the dead zone
    wire signed [31:0] e2 = ex * ex + ey * ey;

    // ---------------- gains ----------------
    wire signed [47:0] pan_mul = cross_z * K_THETA_Q10;
    wire signed [47:0] rch_mul = dr2     * K_R_Q10;
    /* ★ 四舍五入而不是截断：原来 0.79 会被截成 0，导致小误差时通道完全不动
     * （实测 4.5° 角差 -> P=0，闭环在近处死掉）。加半个 LSB 再移位。 */
    wire signed [47:0] pan_rnd = pan_mul + (48'sd1 <<< (SHIFT_PAN - 1));
    wire signed [47:0] rch_rnd = rch_mul + (48'sd1 <<< (SHIFT_RCH - 1));
    wire signed [47:0] pan_sh  = pan_rnd >>> SHIFT_PAN;
    wire signed [47:0] rch_sh  = rch_rnd >>> SHIFT_RCH;

    wire signed [47:0] lim  = LIMIT;
    wire signed [47:0] nlim = -lim;

    wire signed [47:0] pan_cl = (pan_sh > lim) ? lim : (pan_sh < nlim) ? nlim : pan_sh;
    wire signed [47:0] rch_cl = (rch_sh > lim) ? lim : (rch_sh < nlim) ? nlim : rch_sh;

    // ---------------- dead zone / hold ----------------
    wire in_zone = (e2 <= (DEADZONE * DEADZONE));

    reg [7:0] hold_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_pan    <= 16'sd0;
            d_reach  <= 16'sd0;
            o_valid  <= 1'b0;
            o_hold   <= 1'b0;
            hold_cnt <= 8'd0;
        end else begin
            o_valid <= 1'b0;
            if (feat_done) begin
                o_valid <= 1'b1;
                if (!found_r || !found_b) begin
                    // one feature missing: never move on half the information
                    d_pan   <= 16'sd0;
                    d_reach <= 16'sd0;
                    hold_cnt <= 0;
                    o_hold <= 0;
                end else if (in_zone) begin
                    d_pan   <= 16'sd0;
                    d_reach <= 16'sd0;
                    if (hold_cnt < HOLD_N) hold_cnt <= hold_cnt + 1'b1;
                    if (hold_cnt + 1'b1 >= HOLD_N) o_hold <= 1'b1;
                end else begin
                    hold_cnt <= 8'd0;
                    o_hold   <= 1'b0;
                    d_pan    <= pan_cl[15:0];
                    d_reach  <= rch_cl[15:0];
                end
            end
        end
    end

endmodule
