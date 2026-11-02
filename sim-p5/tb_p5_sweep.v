`timescale 1ns / 1ps
// =============================================================================
// tb_p5_sweep.v :  P5 "delay vs closed-loop stability" sweep bench
//
//   plant (same as rtl/tb_servo_closed_loop.v):
//     marker  = pole + r*(cos phi, sin phi)        [pixels]
//     d_pan   -> phi   += d_pan   * DEG_PER_TICK   (0.5 deg/tick at gain 1.0)
//     d_reach -> r     += d_reach * PX_PER_TICK    (0.8 px/tick at gain 1.0)
//     object  = fixed point
//
//   delay_inject sits in the feedback path (vision -> delay -> servo):
//     integrator plant -> servo (P + limit + deadzone) -> delay tau -> plant
//
//   One elaboration runs a whole case list, one CSV line per case:
//     CSV,<mode>,<depth>,<plant_gain>,<seed>,<converged>,<ts_frames>,<e_inf>,<mp>,<pp>,<period_frames>
//   plus one AUX line per case (measured mean/max delay, saturation ratio,
//   start error, used frames, divergence flag, tag) and optional TRC lines.
//
//   case file "cases.txt" (ASCII, all-integer fields, 11 per line):
//     mode depth gain_milli seed nframes obj_x obj_y r0_milli phi0_milli trace tag
//   gain_milli = plant gain * 1000 ; r0/phi0 also * 1000.  DEG_PER_TICK is
//   derived as 0.5*gain/0.8 so that one sweep knob scales the whole loop gain.
//
//   Servo gain: default 32 (override with  xvlog -d KT=.. -d KR=..).
//   Why: the shipped K_THETA_Q10=4 puts the pan quantisation step at 11.7 px
//   at R=350, i.e. larger than the 6 px deadzone -> the loop parks outside the
//   deadzone (ref tb really does stop at e=60 px, FAIL).  See 汇总.md.
// =============================================================================
module tb_p5_sweep;

    localparam real PI_R = 3.14159265358979;
`ifndef DZ_P
    `define DZ_P 6
`endif
    localparam integer DZP = `DZ_P;          // servo deadzone (px)
    localparam real DZ   = DZP;             // same value used for ts_frames
`ifndef KT
    `define KT 32
`endif
`ifndef KR
    `define KR 32
`endif
    localparam integer KT_ = `KT;
    localparam integer KR_ = `KR;

    // ------------------------------------------------------------------ config
    real     PLANT_PX;       // pixels per reach tick
    real     PLANT_DEG;      // degrees per pan tick
    real     POLE_X, POLE_Y;
    integer  OBJ_X, OBJ_Y;
    integer  NF, TRACE;
    real     R0, PHI0;
    integer  c_mode, c_depth, c_seed, c_tag;
    integer  c_A;
    real     c_gain, c_px, c_deg;
    reg [15:0] tb_lfsr;
    integer  uu, vv;

    // 16 bit Galois LFSR step (same polynomial as delay_inject, own state)
    function [15:0] lstep;
        input [15:0] s;
        input integer n;
        integer k;
        reg [15:0] t;
        begin
            t = s;
            for (k = 0; k < n; k = k + 1)
                t = {t[14:0], 1'b0} ^ (t[15] ? 16'h6801 : 16'd0);
            lstep = t;
        end
    endfunction

    // ------------------------------------------------------------------ signals
    reg clk = 0, rst_n = 0, feat_done = 0;
    always #5 clk = ~clk;            // 100 MHz, one frame = 5 clk = 50 ns

    reg [15:0] cx_r = 0, cy_r = 0, cx_b = 0, cy_b = 0;
    reg [1:0]  d_mode  = 2'd0;       // 0 bypass / 1 fixed / 2 LFSR jitter
    reg [3:0]  d_depth = 4'd0;

    wire        dl_valid, dl_found_r, dl_found_b;
    wire [15:0] dl_cx_r, dl_cy_r, dl_cx_b, dl_cy_b;
    wire [3:0]  dl_delay;

    wire signed [15:0] d_pan, d_reach;
    wire o_valid, o_hold;

    // ------------------------------------------------------------------ DUTs
    delay_inject #(
        .D_MAX(14), .D_FIXED(0), .D_MAX_JIT(14),
        .LFSR_SEED(16'hACE1), .JIT_STEP(8), .MODE(2'd0)
    ) u_dly (
        .clk(clk), .rst_n(rst_n),
        .feat_done(feat_done),
        .cx_r(cx_r), .cy_r(cy_r), .cx_b(cx_b), .cy_b(cy_b),
        .found_r(1'b1), .found_b(1'b1),
        .d_mode(d_mode), .d_depth(d_depth),
        .o_valid(dl_valid),
        .o_cx_r(dl_cx_r), .o_cy_r(dl_cy_r), .o_cx_b(dl_cx_b), .o_cy_b(dl_cy_b),
        .o_found_r(dl_found_r), .o_found_b(dl_found_b),
        .o_delay(dl_delay)
    );

    servo_polar #(
        .C_X(400), .C_Y(240),
        .K_THETA_Q10(KT_), .K_R_Q10(KR_),
        .DEADZONE(DZP), .LIMIT(4), .HOLD_N(5),
        .SHIFT_PAN(15), .SHIFT_RCH(16)
    ) u_srv (
        .clk(clk), .rst_n(rst_n),
        .feat_done(dl_valid),
        .cx_r(dl_cx_r), .cy_r(dl_cy_r),
        .cx_b(dl_cx_b), .cy_b(dl_cy_b),
        .found_r(dl_found_r), .found_b(dl_found_b),
        .cfg_cx(16'd400), .cfg_cy(16'd240),
        .d_pan(d_pan), .d_reach(d_reach),
        .o_valid(o_valid), .o_hold(o_hold)
    );

    // ------------------------------------------------------------------ state
    real    r_s, phi_s;
    real    e_arr [0:4095];      // |marker-object|            (as specified)
    real    s_arr [0:4095];      // signed radial error cx_r-OBJ_X (collinear cases)
    integer nrec;
    integer m_ts, m_conv;
    real    m_einf, m_mp, m_pp, m_per;
    real    m_sper, m_spp, m_sinf;
    real    a_mean_dly, a_max_dly, a_sat, a_e0, a_emax, a_efin;
    integer a_div, a_nuse;

    // ------------------------------------------------------------------ tasks
    task do_reset;
        begin
            rst_n = 1'b0;
            feat_done = 1'b0;
            repeat (6) @(negedge clk);
            rst_n = 1'b1;
            repeat (4) @(negedge clk);
        end
    endtask

    // one frame: feat_done -> delay_inject (3 clk) -> o_valid -> servo (1 clk)
    task frame_tick;
        begin
            @(negedge clk); feat_done = 1'b1;
            @(negedge clk); feat_done = 1'b0;
            repeat (3) @(negedge clk);      // d_pan / d_reach valid here
        end
    endtask

    task emit_metrics;
        integer i, n, n10, n3, cnt;
        real s, mn, mx, y0, y1, xc, firstx, lastx;
        begin
            n = nrec;
            m_conv = 0; m_ts = -1;
            m_einf = 0.0; m_mp = 0.0; m_pp = 0.0; m_per = 0.0; a_efin = 0.0;
            if (n > 0) begin
                // ts_frames: last frame from which e stays inside the deadzone
                i = n - 1;
                while (i >= 0 && e_arr[i] <= DZ) begin
                    m_ts = i;
                    i = i - 1;
                end
                // e_inf: mean over the last 10 % of frames
                n10 = n / 10; if (n10 < 1) n10 = 1;
                s = 0.0;
                for (i = n - n10; i < n; i = i + 1) s = s + e_arr[i];
                m_einf = s / $itor(n10);
                // mp: max over the second half minus e_inf
                mx = -1.0e30;
                for (i = n / 2; i < n; i = i + 1) if (e_arr[i] > mx) mx = e_arr[i];
                m_mp = mx - m_einf;
                // pp: peak-to-peak over the last third
                n3 = n / 3; if (n3 < 3) n3 = 3;
                mn = 1.0e30; mx = -1.0e30;
                for (i = n - n3; i < n; i = i + 1) begin
                    if (e_arr[i] < mn) mn = e_arr[i];
                    if (e_arr[i] > mx) mx = e_arr[i];
                end
                m_pp = mx - mn;
                // period: mean interval of positive zero crossings of e-e_inf
                //         inside the last third (0 if fewer than 2 crossings)
                cnt = 0; firstx = 0.0; lastx = 0.0;
                for (i = n - n3 + 1; i < n; i = i + 1) begin
                    y0 = e_arr[i-1] - m_einf;
                    y1 = e_arr[i]   - m_einf;
                    if (y0 <= 0.0 && y1 > 0.0) begin
                        xc = $itor(i-1) + (-y0) / (y1 - y0);
                        if (cnt == 0) begin firstx = xc; lastx = xc; end
                        else lastx = xc;
                        cnt = cnt + 1;
                    end
                end
                if (cnt >= 2) m_per = (lastx - firstx) / $itor(cnt - 1);
                else          m_per = 0.0;
                a_efin = e_arr[n-1];
            end
            if (m_ts >= 0 && m_pp <= DZ && m_einf <= DZ) m_conv = 1;
        end
    endtask

    // Signed-error period: same construction as the |e| period, but on the
    // signed radial error.  |e| is a rectified signal, so its period is half
    // of the signed one - this is what makes 2*tau+1 look like "not 4*tau".
    task emit_signed;
        integer i, n, n10, n3, cnt;
        real s, mn, mx, y0, y1, xc, firstx, lastx;
        begin
            n = nrec;
            m_sper = 0.0; m_spp = 0.0; m_sinf = 0.0;
            if (n > 0) begin
                n10 = n / 10; if (n10 < 1) n10 = 1;
                s = 0.0;
                for (i = n - n10; i < n; i = i + 1) s = s + s_arr[i];
                m_sinf = s / $itor(n10);
                n3 = n / 3; if (n3 < 3) n3 = 3;
                mn = 1.0e30; mx = -1.0e30;
                for (i = n - n3; i < n; i = i + 1) begin
                    if (s_arr[i] < mn) mn = s_arr[i];
                    if (s_arr[i] > mx) mx = s_arr[i];
                end
                m_spp = mx - mn;
                cnt = 0; firstx = 0.0; lastx = 0.0;
                for (i = n - n3 + 1; i < n; i = i + 1) begin
                    y0 = s_arr[i-1] - m_sinf;
                    y1 = s_arr[i]   - m_sinf;
                    if (y0 <= 0.0 && y1 > 0.0) begin
                        xc = $itor(i-1) + (-y0) / (y1 - y0);
                        if (cnt == 0) begin firstx = xc; lastx = xc; end
                        else lastx = xc;
                        cnt = cnt + 1;
                    end
                end
                if (cnt >= 2) m_sper = (lastx - firstx) / $itor(cnt - 1);
                else          m_sper = 0.0;
            end
        end
    endtask

    task run_case;
        integer i;
        real ex, ey, ee;
        begin
            PLANT_PX  = c_px;
            PLANT_DEG = c_deg;
            // mode 3 = tb-side symmetric jitter: reuse delay_inject's fixed
            // (mode 1) datapath and just drive d_depth per frame
            d_mode    = (c_mode == 3) ? 2'd1 : c_mode[1:0];
            d_depth   = c_depth[3:0];
            r_s   = R0;
            phi_s = PHI0 * PI_R / 180.0;
            nrec  = 0;
            e_arr[0] = 0.0;

            do_reset;
            u_dly.lfsr = c_seed[15:0] + 16'hACE1;   // sim-only hierarchical poke
            tb_lfsr    = c_seed[15:0] * 16'h0100 + 16'h1234;   // tb jitter source
            if (tb_lfsr == 16'd0) tb_lfsr = 16'h1234;

            cx_r = $rtoi(POLE_X + r_s * $cos(phi_s));
            cy_r = $rtoi(POLE_Y + r_s * $sin(phi_s));
            cx_b = OBJ_X;
            cy_b = OBJ_Y;

            ex = $itor(OBJ_X) - $itor(cx_r);
            ey = $itor(OBJ_Y) - $itor(cy_r);
            a_e0 = $sqrt(ex*ex + ey*ey);

            a_mean_dly = 0.0; a_max_dly = 0.0; a_sat = 0.0;
            a_div = 0; a_emax = 0.0; a_efin = 0.0;

            for (i = 0; i < NF; i = i + 1) begin
                // mode 3: symmetric jitter source, delay = MEAN + U, U uniform
                // on [-A,+A]  -> mean is fixed by construction, sigma = A only.
                // delay_inject still carries the ring buffer; only d_depth is
                // driven per frame, so the RTL is untouched.
                if (c_mode == 3) begin
                    tb_lfsr = lstep(tb_lfsr, 8);
                    uu = (tb_lfsr % (2 * c_A + 1)) - c_A;
                    vv = c_depth + uu;
                    if (vv > 14) vv = 14;
                    if (vv < 0)  vv = 0;
                    d_depth = vv[3:0];
                end
                frame_tick;

                a_mean_dly = a_mean_dly + $itor(dl_delay);
                if ($itor(dl_delay) > a_max_dly) a_max_dly = $itor(dl_delay);
                if (($itor(d_pan) >= 4.0) || ($itor(d_pan) <= -4.0) ||
                    ($itor(d_reach) >= 4.0) || ($itor(d_reach) <= -4.0))
                    a_sat = a_sat + 1.0;

                // ---- plant ----
                phi_s = phi_s + $itor(d_pan) * (PLANT_DEG * PI_R / 180.0);
                r_s   = r_s   + $itor(d_reach) * PLANT_PX;
                if (r_s < 5.0)    r_s = 5.0;
                if (r_s > 3000.0) r_s = 3000.0;
                cx_r = $rtoi(POLE_X + r_s * $cos(phi_s));
                cy_r = $rtoi(POLE_Y + r_s * $sin(phi_s));

                ex = $itor(OBJ_X) - $itor(cx_r);
                ey = $itor(OBJ_Y) - $itor(cy_r);
                ee = $sqrt(ex*ex + ey*ey);
                if (ee != ee) ee = 9999.0;          // NaN guard
                e_arr[i] = ee;
                s_arr[i] = $itor(cx_r) - $itor(OBJ_X);   // signed radial error
                nrec = i + 1;
                if (ee > a_emax) a_emax = ee;

                if (TRACE != 0)
                    $display("TRC,%0d,%0d,%0.4f,%0d,%0d,%0d,%0.2f",
                             c_tag, i, ee, d_pan, d_reach, dl_delay, (phi_s*180.0/PI_R));
                else if (i % 100 == 0)
                    $display("  tag=%0d f%0d e=%0.2f dp=%0d dr=%0d dly=%0d r=%0.1f phi=%0.2f",
                             c_tag, i, ee, d_pan, d_reach, dl_delay, r_s, phi_s*180.0/PI_R);

                if (ee > 2500.0 || r_s >= 2999.0) begin
                    a_div = 1;
                    i = NF;                          // break
                end
            end

            a_nuse = nrec;
            if (a_nuse > 0) begin
                a_mean_dly = a_mean_dly / $itor(a_nuse);
                a_sat      = a_sat / $itor(a_nuse);
            end
            emit_metrics;
            emit_signed;

            $display("CSV,%0d,%0d,%0.3f,%0d,%0d,%0d,%0.4f,%0.4f,%0.4f,%0.2f",
                     c_mode, c_depth, c_gain, c_seed, m_conv, m_ts,
                     m_einf, m_mp, m_pp, m_per);
            $display("AUX,%0d,%0d,%0.3f,%0d,%0.2f,%0.2f,%0.3f,%0.3f,%0.3f,%0d,%0d,%0.3f,%0d,%0.2f,%0.2f",
                     c_mode, c_depth, c_gain, c_seed,
                     a_mean_dly, a_max_dly, a_sat, a_e0, a_emax, a_nuse, a_div, a_efin, c_tag,
                     m_sper, m_spp);
            // sigma sweep: mean-delay / jitter-amplitude / seed + the metrics
            if (c_mode == 3) begin
                $display("SCSV,%0d,%0d,%0d,%0d,%0d,%0.4f,%0.4f,%0.2f",
                         c_depth, c_A, c_seed, m_conv, m_ts, m_mp, m_pp, m_per);
                if (c_A == 0)
                    $display("REF,mean=%0d,A=0,seed=%0d,conv=%0d,ts=%0d,e_inf=%0.4f,mp=%0.4f,pp=%0.4f,per=%0.2f,mean_dly=%0.3f",
                             c_depth, c_seed, m_conv, m_ts, m_einf, m_mp, m_pp, m_per, a_mean_dly);
            end
        end
    endtask

    // ------------------------------------------------------------------ main
    integer fd, rc, ncase, done;
    integer f_mode, f_depth, f_gainm, f_seed, f_nf, f_ox, f_oy, f_r0m, f_phim, f_tr, f_tag, f_jitA;
    initial begin
        POLE_X = 400.0; POLE_Y = 240.0;
        OBJ_X = 750; OBJ_Y = 240;
        NF = 400; TRACE = 0; R0 = 160.0; PHI0 = 0.0;
        PLANT_PX = 0.8; PLANT_DEG = 0.5;

        $display("=== tb_p5_sweep KT=%0d KR=%0d ===", KT_, KR_);

        fd = $fopen("cases.txt", "r");
        if (fd == 0) begin
            $display("ERROR: cannot open cases.txt");
            $finish;
        end
        ncase = 0; done = 0;
        while (done == 0) begin
            rc = $fscanf(fd, "%d %d %d %d %d %d %d %d %d %d %d %d",
                         f_mode, f_depth, f_gainm, f_seed, f_nf,
                         f_ox, f_oy, f_r0m, f_phim, f_tr, f_tag, f_jitA);
            if (rc != 12) done = 1;
            else begin
                c_mode   = f_mode;
                c_depth  = f_depth;
                c_A      = f_jitA;
                c_gain   = $itor(f_gainm) / 1000.0;
                c_px     = c_gain;
                c_deg    = 0.5 * c_gain / 0.8;
                c_seed   = f_seed;
                c_tag    = f_tag;
                NF       = (f_nf > 4096) ? 4096 : f_nf;
                OBJ_X    = f_ox;
                OBJ_Y    = f_oy;
                R0       = $itor(f_r0m) / 1000.0;
                PHI0     = $itor(f_phim) / 1000.0;
                TRACE    = f_tr;
                run_case;
                ncase = ncase + 1;
            end
        end
        $fclose(fd);
        $display("=== tb_p5_sweep done : %0d cases ===", ncase);
        $finish;
    end

    initial begin
        #60000000;                       // 60 ms of sim time
        $display("=== TIMEOUT ===");
        $finish;
    end

endmodule
