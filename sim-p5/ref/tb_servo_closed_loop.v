`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// tb_servo_closed_loop : closed-loop check of servo_polar against a simple plant
//
//   plant (image space):
//     marker position: c + r*(cos(phi), sin(phi))          [pixels]
//     d_pan   -> phi += d_pan   * DEG_PER_TICK
//     d_reach -> r   += d_reach * PX_PER_TICK
//   object: fixed at (450, 300)
//
//   expected: |e| decreases monotonically (apart from rounding) and settles
//             inside DEADZONE; o_hold asserts afterwards.
// -----------------------------------------------------------------------------
module tb_servo_closed_loop;

    localparam real C_X_ = 400.0;
    localparam real C_Y_ = 240.0;
    localparam real DEG_PER_TICK = 0.5;     // degrees per pan tick
    localparam real RAD_PER_TICK = DEG_PER_TICK * 3.14159265 / 180.0;
    localparam real PX_PER_TICK  = 0.8;     // pixels per reach tick

    reg clk = 0, rst_n = 0;
    reg feat_done = 0;
    reg [15:0] cx_r = 0, cy_r = 0, cx_b = 0, cy_b = 0;
    reg found_r = 1, found_b = 1;

    wire signed [15:0] d_pan, d_reach;
    wire o_valid, o_hold;

    real r_m, phi;
    real e_len, e_len_prev;
    integer i;
    integer errors = 0;
    integer converged_at = -1;
    real max_e = 0.0;

    always #5 clk = ~clk;

    servo_polar #(
        .C_X(400), .C_Y(240),
        .K_THETA_Q10(4), .K_R_Q10(4),
        .DEADZONE(6), .LIMIT(4), .HOLD_N(5)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .feat_done(feat_done),
        .cx_r(cx_r), .cy_r(cy_r),
        .cx_b(cx_b), .cy_b(cy_b),
        .found_r(found_r), .found_b(found_b),
        .cfg_cx(16'd400), .cfg_cy(16'd240),
        .d_pan(d_pan), .d_reach(d_reach),
        .o_valid(o_valid), .o_hold(o_hold)
    );

    task one_frame;
        begin
            @(negedge clk);
            feat_done = 1'b1;
            @(negedge clk);
            feat_done = 1'b0;
            @(negedge clk);      // o_valid / outputs valid here
        end
    endtask

    initial begin
        $display("=== tb_servo_closed_loop start ===");

        r_m = 160.0;
        phi = 0.0;
        cx_b = 450; cy_b = 300;

        rst_n = 0;
        repeat (10) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // initial error
        cx_r = $rtoi(C_X_ + r_m * $cos(phi));
        cy_r = $rtoi(C_Y_ + r_m * $sin(phi));
        e_len = $sqrt(($itor(cx_b) - $itor(cx_r)) * ($itor(cx_b) - $itor(cx_r)) +
                      ($itor(cy_b) - $itor(cy_r)) * ($itor(cy_b) - $itor(cy_r)));
        $display("  start: marker=(%0d,%0d) object=(%0d,%0d) |e|=%0.1f",
                 cx_r, cy_r, cx_b, cy_b, e_len);
        e_len_prev = e_len;
        max_e = e_len;

        for (i = 0; i < 200; i = i + 1) begin
            one_frame;

            // apply commands to the plant
            phi = phi + $itor(d_pan) * RAD_PER_TICK;
            r_m = r_m + $itor(d_reach) * PX_PER_TICK;
            if (r_m < 5.0) r_m = 5.0;

            // new marker pixel position
            cx_r = $rtoi(C_X_ + r_m * $cos(phi));
            cy_r = $rtoi(C_Y_ + r_m * $sin(phi));

            e_len = $sqrt(($itor(cx_b) - $itor(cx_r)) * ($itor(cx_b) - $itor(cx_r)) +
                          ($itor(cy_b) - $itor(cy_r)) * ($itor(cy_b) - $itor(cy_r)));
            if (e_len > max_e) max_e = e_len;

            if (i < 40 || i % 20 == 0)
                $display("  frame %3d : marker=(%0d,%0d) r=%8.2f phi=%7.2f  |e|=%0.1f  dp=%0d dr=%0d itor=%0.1f",
                         i, cx_r, cy_r, r_m, phi * 180.0 / 3.14159265, e_len, d_pan, d_reach, $itor(d_reach));

            if (converged_at < 0 && e_len <= 6.0) converged_at = i;

            // divergence guard
            if (e_len > 200.0) begin
                $display("  [FAIL] diverged at frame %0d (|e|=%0.1f)", i, e_len);
                errors = errors + 1;
                i = 1000;
            end
        end

        $display("  final: |e|=%0.1f  max|e|=%0.1f  hold=%b  converged_at=%0d",
                 e_len, max_e, o_hold, converged_at);

        if (converged_at < 0) begin
            $display("  [FAIL] did not converge into the dead zone");
            errors = errors + 1;
        end
        if (converged_at > 0 && converged_at > 80) begin
            $display("  [FAIL] too slow to converge (%0d frames)", converged_at);
            errors = errors + 1;
        end
        if (!o_hold) begin
            $display("  [FAIL] o_hold did not assert after convergence");
            errors = errors + 1;
        end
        if (max_e > e_len * 1.05 + 1.0) begin
            $display("  [WARN] error grew during the run (start %0.1f -> max %0.1f)", e_len, max_e);
        end

        if (errors == 0) $display("=== PASS : converged in %0d frames, settled |e|=%0.1f ===",
                                  converged_at, e_len);
        else             $display("=== FAIL : %0d problems ===", errors);
        $finish;
    end

    initial begin
        #5000000;
        $display("=== FAIL : timeout ===");
        $finish;
    end

endmodule
