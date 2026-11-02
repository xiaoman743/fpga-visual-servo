`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// vision_features : RGB565 stream -> two centroids (red = marker, blue = object)
//
//   camera side : i_vsync / i_valid / i_rgb  (cam_pclk domain)
//   result side : o_valid (1-cycle pulse per frame) + 2 centroids + found flags
//
//   X_FLIP mirrors the x coordinate so the host code does not have to care how
//   the camera is mounted. The flip is applied here, once, so the OSD and the
//   servo always work in the same frame of reference.
//
//   Integration (top level of 37_ov5640_lcd):
//     .clk     (cam_pclk)
//     .i_vsync (cmos_frame_vsync)
//     .i_valid (cmos_frame_valid)
//     .i_rgb   (wrdata)
// -----------------------------------------------------------------------------
module vision_features #(
    parameter [7:0] MARGIN   = 8'd40,
    parameter [7:0] MIN_RED  = 8'd90,
    parameter [7:0] MIN_BLUE = 8'd70,
    parameter integer H_PIXEL = 800,
    parameter integer V_PIXEL = 480,
    parameter integer X_FLIP  = 1     // 1 = mirror x (camera mounted rotated)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_vsync,
    input  wire        i_valid,
    input  wire [15:0] i_rgb,

    output wire        o_valid,
    output wire [15:0] o_x_red,
    output wire [15:0] o_y_red,
    output wire        o_found_red,
    output wire [15:0] o_x_blue,
    output wire [15:0] o_y_blue,
    output wire        o_found_blue
);

    wire        m_valid;
    wire        m_red;
    wire        m_blue;

    wire [15:0] x_red_raw;
    wire [15:0] x_blue_raw;

    rgb_color_mask #(
        .MARGIN   (MARGIN),
        .MIN_RED  (MIN_RED),
        .MIN_BLUE (MIN_BLUE)
    ) u_mask (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_valid (i_valid),
        .i_rgb   (i_rgb),
        .o_valid (m_valid),
        .o_red   (m_red),
        .o_blue  (m_blue)
    );

    centroid_acc #(
        .H_PIXEL (H_PIXEL),
        .V_PIXEL (V_PIXEL)
    ) u_cen (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_vsync      (i_vsync),
        .i_valid      (m_valid),
        .i_red        (m_red),
        .i_blue       (m_blue),
        .o_valid      (o_valid),
        .o_x_red      (x_red_raw),
        .o_y_red      (o_y_red),
        .o_found_red  (o_found_red),
        .o_x_blue     (x_blue_raw),
        .o_y_blue     (o_y_blue),
        .o_found_blue (o_found_blue)
    );

    assign o_x_red  = X_FLIP ? (H_PIXEL - 1 - x_red_raw)  : x_red_raw;
    assign o_x_blue = X_FLIP ? (H_PIXEL - 1 - x_blue_raw) : x_blue_raw;

endmodule
