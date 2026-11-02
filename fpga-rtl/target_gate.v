`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// target_gate : 目标（蓝色物体）位置锁定 —— 抗遮挡 / 抗质心抖动
//
// 为什么需要它（2026-10-28 现场问题）：
//   机械爪接近/夹住蓝块时，会把蓝色像素挡住一部分 → 质心朝"可见部分"跳
//   → 伺服以为目标动了 → 底座来回摆 → 越靠近抖得越厉害（追自己的影子）。
//   这类误差不是噪声，是**结构性**的：只要有遮挡就一定有跳变。
//
// 策略（用户提的"前置判断"）：锁存 + 迟滞 + 重捕获
//   ① 新目标首次出现：直接锁定
//   ② 之后每帧比较：|新质心 - 锁定值| <= LOCK_R  → 认为还是同一块，输出锁定值（不动）
//      （这一条同时吃掉质心 ±1~2px 的抖动和部分遮挡造成的小位移）
//   ③ 连续 FAR_N 帧都超出 LOCK_R → 认为目标真的被移动了，重新锁定到新位置
//      用"连续 N 帧"而不是单帧判定：单帧跳变（夹爪边缘进/出画面）不该改目标
//   ④ 物体短暂丢失（found=0）：保持锁定位置最多 LOST_N 帧，仍然输出 found=1，
//      避免伺服因为一帧丢失就整帧清零；超过 LOST_N 才把 found 拉低
//
// ⚠️ 已知边界：这是**半径锁定**，不是真正的"准星还在蓝色像素内"（那需要 bbox）。
//    蓝块很大（~100px）时 LOCK_R=24 只能覆盖不到半径一半，夹爪从边上啃掉一部分
//    仍可能超过阈值。真要做 bbox 需要 centroid_acc 顺带输出 min/max。
// -----------------------------------------------------------------------------
module target_gate #(
    parameter integer LOCK_R   = 24,   // 锁定半径（px，曼哈顿距离）
    parameter integer FAR_N    = 4,    // 连续多少帧超出才重捕获（≈150ms @26.7fps）
    parameter integer LOST_N   = 3,    // 丢失多少帧内仍输出锁定值
    parameter integer X_MAX    = 799,
    parameter integer Y_MAX    = 479
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        feat_done,
    input  wire [15:0] cx_b,
    input  wire [15:0] cy_b,
    input  wire        found_b,
    output reg  [15:0] gx_b,
    output reg  [15:0] gy_b,
    output reg         gfound_b,
    output reg         o_reacquired,   // 一帧脉冲：刚刚重捕获（可接示波器/计数）
    output reg         o_gated         // 一帧脉冲：本帧被锁定拒绝（诊断用）
);

    reg        locked;
    reg [15:0] lx, ly;
    reg [3:0]  far_cnt;
    reg [3:0]  lost_cnt;

    wire [15:0] dx = (cx_b > lx) ? (cx_b - lx) : (lx - cx_b);
    wire [15:0] dy = (cy_b > ly) ? (cy_b - ly) : (ly - cy_b);
    wire [16:0] dman = {1'b0, dx} + {1'b0, dy};   // 曼哈顿距离，省一个乘法器
    wire        near = (dman <= LOCK_R);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            locked        <= 1'b0;
            lx            <= 16'd0;
            ly            <= 16'd0;
            far_cnt       <= 4'd0;
            lost_cnt      <= 4'd0;
            gx_b          <= 16'd0;
            gy_b          <= 16'd0;
            gfound_b      <= 1'b0;
            o_reacquired  <= 1'b0;
            o_gated       <= 1'b0;
        end else begin
            o_reacquired <= 1'b0;
            o_gated      <= 1'b0;

            if (feat_done) begin
                if (!found_b) begin
                    /* ---- 本帧没检到 ---- */
                    far_cnt <= 4'd0;
                    // Coordinates may be remembered, but absent is never fresh.
                    // Saturate and release the lock: no periodic found reassertion.
                    gfound_b <= 1'b0;
                    gx_b <= lx;
                    gy_b <= ly;
                    if (lost_cnt < LOST_N) lost_cnt <= lost_cnt + 4'd1;
                    if (lost_cnt >= LOST_N-1) locked <= 1'b0;
                end else if (!locked) begin
                    /* ---- 首次锁定 ---- */
                    locked   <= 1'b1;
                    lx       <= cx_b;
                    ly       <= cy_b;
                    gx_b     <= cx_b;
                    gy_b     <= cy_b;
                    gfound_b <= 1'b1;
                    far_cnt  <= 4'd0;
                    lost_cnt <= 4'd0;
                end else if (near) begin
                    /* ---- 还在同一块区域内：**不动**（吃掉抖动/小遮挡）---- */
                    far_cnt  <= 4'd0;
                    lost_cnt <= 4'd0;
                    gx_b     <= lx;
                    gy_b     <= ly;
                    gfound_b <= 1'b1;
                    o_gated  <= 1'b1;
                end else begin
                    /* ---- 跑出锁定半径：连续 FAR_N 帧才认 ---- */
                    lost_cnt <= 4'd0;
                    if (far_cnt + 4'd1 >= FAR_N) begin
                        lx           <= cx_b;
                        ly           <= cy_b;
                        far_cnt      <= 4'd0;
                        gx_b         <= cx_b;
                        gy_b         <= cy_b;
                        gfound_b     <= 1'b1;
                        o_reacquired <= 1'b1;
                    end else begin
                        far_cnt  <= far_cnt + 4'd1;
                        gx_b     <= lx;      // 过渡期仍用旧锁定值（不追）
                        gy_b     <= ly;
                        gfound_b <= 1'b1;
                        o_gated  <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
