`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// centroid_acc : accumulate (x, y, count) for red and blue, then divide at EOF
//   - x/y counters are driven by i_valid (H_PIXEL pixels per line)
//   - accumulators are cleared on the vsync rising edge
//   - division is a sequential subtract-and-count FSM:
//       worst case H_PIXEL + V_PIXEL cycles per feature, done during the next
//       frame's blanking; negligible vs the 33 ms frame period
//   - o_valid is a 1-cycle pulse when all four coordinates are ready
// -----------------------------------------------------------------------------
module centroid_acc #(
    parameter integer H_PIXEL = 800,
    parameter integer V_PIXEL = 480,
    parameter integer MIN_PIXELS = 16
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        i_vsync,     // frame boundary (= cmos_frame_vsync)
    input  wire        i_valid,     // pixel valid
    input  wire        i_red,
    input  wire        i_blue,
    output reg         o_valid,
    output reg  [15:0] o_x_red,
    output reg  [15:0] o_y_red,
    output reg         o_found_red,
    output reg  [15:0] o_x_blue,
    output reg  [15:0] o_y_blue,
    output reg         o_found_blue
);

    // ---------------- frame boundary ----------------
    reg vsync_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) vsync_d <= 1'b0;
        else        vsync_d <= i_vsync;

    wire frame_start = i_vsync & ~vsync_d;

    // ---------------- pixel coordinates ----------------
    reg [10:0] x;
    reg [9:0]  y;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 11'd0;
            y <= 10'd0;
        end else if (i_vsync) begin
            x <= 11'd0;
            y <= 10'd0;
        end else if (i_valid) begin
            if (x == H_PIXEL - 1) begin
                x <= 11'd0;
                y <= y + 1'b1;
            end else begin
                x <= x + 1'b1;
            end
        end
    end

    // ---------------- per-frame accumulators ----------------
    reg [31:0] sx0, sy0, sx1, sy1;   // 0 = red, 1 = blue
    reg [23:0] c0,  c1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sx0 <= 0; sy0 <= 0; c0 <= 0;
            sx1 <= 0; sy1 <= 0; c1 <= 0;
        end else if (frame_start) begin
            sx0 <= 0; sy0 <= 0; c0 <= 0;
            sx1 <= 0; sy1 <= 0; c1 <= 0;
        end else if (i_valid) begin
            if (i_red) begin
                sx0 <= sx0 + x;
                sy0 <= sy0 + y;
                c0  <= c0 + 1'b1;
            end
            if (i_blue) begin
                sx1 <= sx1 + x;
                sy1 <= sy1 + y;
                c1  <= c1 + 1'b1;
            end
        end
    end

    // ---------------- latched copies + division FSM ----------------
    localparam S_IDLE = 2'd0, S_RUN = 2'd1, S_DONE = 2'd2;

    reg [1:0]  st;
    reg [1:0]  ch;          // 0:red_x 1:red_y 2:blue_x 3:blue_y
    reg [31:0] lx0, ly0, lx1, ly1;
    reg [23:0] lc0, lc1;
    reg [31:0] rem;
    reg [15:0] q;

    wire [31:0] curn = ch[1] ? {8'd0, lc1} : {8'd0, lc0};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; ch <= 2'd0; rem <= 0; q <= 0; o_valid <= 1'b0;
            lx0 <= 0; ly0 <= 0; lx1 <= 0; ly1 <= 0; lc0 <= 0; lc1 <= 0;
            o_x_red <= 0; o_y_red <= 0; o_x_blue <= 0; o_y_blue <= 0;
            o_found_red <= 1'b0; o_found_blue <= 1'b0;
        end else begin
            o_valid <= 1'b0;

            case (st)
            // ---- latch the last frame and start dividing ----
            S_IDLE: begin
                if (frame_start) begin
                    lx0 <= sx0; ly0 <= sy0; lc0 <= c0;
                    lx1 <= sx1; ly1 <= sy1; lc1 <= c1;
                    ch  <= 2'd0;
                    rem <= sx0;
                    q   <= 16'd0;
                    o_found_red  <= (c0 >= MIN_PIXELS);
                    o_found_blue <= (c1 >= MIN_PIXELS);
                    st  <= S_RUN;
                end
            end
            // ---- sequential division: q = sum / count ----
            S_RUN: begin
                if (curn == 32'd0) begin
                    // feature absent in that frame -> output 0 so the host does not
                    // mistake a stale centroid for a live one
                    case (ch)
                        2'd0: o_x_red  <= 16'd0;
                        2'd1: o_y_red  <= 16'd0;
                        2'd2: o_x_blue <= 16'd0;
                        default: o_y_blue <= 16'd0;
                    endcase
                    q  <= 16'd0;
                    ch <= ch + 2'd1;
                    case (ch)
                        2'd0: rem <= ly0;
                        2'd1: rem <= lx1;
                        2'd2: rem <= ly1;
                        default: st <= S_DONE;
                    endcase
                end else if (rem >= curn) begin
                    rem <= rem - curn;
                    q   <= q + 16'd1;
                end else begin
                    case (ch)
                        2'd0: o_x_red  <= q;
                        2'd1: o_y_red  <= q;
                        2'd2: o_x_blue <= q;
                        default: o_y_blue <= q;
                    endcase
                    ch <= ch + 2'd1;
                    q  <= 16'd0;
                    case (ch)
                        2'd0: rem <= ly0;
                        2'd1: rem <= lx1;
                        2'd2: rem <= ly1;
                        default: st <= S_DONE;
                    endcase
                end
            end
            // ---- one-cycle ready pulse ----
            S_DONE: begin
                o_valid <= 1'b1;
                st      <= S_IDLE;
            end
            endcase
        end
    end

endmodule
