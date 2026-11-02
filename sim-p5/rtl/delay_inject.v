`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// delay_inject : 把一帧特征整体延迟 D 帧之后，再交给 servo_polar
//
//   P5「确定性延迟」差异化的硬件支点。A/B 两组共用同一份伺服 RTL，只切这个模块的
//   两个寄存器（d_mode / d_depth），所以两组数据可比。
//
//   数据流：
//     vision_features ──feat_done + 66 bit──> delay_inject ──> servo_polar
//                                                      └──> feat_uart（建议接延迟后，
//                                                           日志才是伺服"所见"）
//   时钟：只有一个 clk（= cam_pclk，和 servo_polar 同域）。
//         只有 d_mode/d_depth 来自 cmd_parse 的 clk_50m 域，因此这两位在本模块内
//         做 2 级同步（与 servo_polar 同步 cfg_cx/cfg_cy 的做法一致）。
//
//   一帧载荷 = 66 bit：cx_r(16) + cy_r(16) + cx_b(16) + cy_b(16) + found_r(1) + found_b(1)
//
//   ── 为什么用环形缓冲，不用移位寄存器堆 ──────────────────────────────
//   移位堆要 66 bit × 8 级 = 528 个 FF，且每帧 528 个 FF 全在翻转（功耗）；
//   环缓只要 9 格 × 66 bit，写一格、读一格，缓冲本体在 BRAM 里，占 0 个 FF。
//   实测（xc7a100tfgg484-2 / Vivado 2020.2 / 18 ns 约束）：
//     65 LUT / 123 FF / 1 BRAM / 1 DSP，WNS = +10.0 ns，0 latch，综合 0 warning。
//   其中 123 个 FF 里有 66 个是 o_* 输出寄存器、16 个是 LFSR —— 延迟深度再大也不涨。
//   这就是本模块唯一值得说的结构取舍：**延迟档位从 1 帧拉到 8 帧，面积不变**。
//
//   ── 时序（feat_done 到输出固定晚 3 拍，与 D 无关）────────────────────
//     T    : feat_done 高，本帧载荷与目标延迟 d_eff 在总线上
//     T+1  : 拍1 —— 锁存读地址、写 RAM、推进写指针/帧计数
//     T+2  : 拍2 —— RAM 出数（mem_rd 有效）、有效屏蔽位就位
//     T+3  : 拍3 —— o_* 寄存输出，o_valid 拉高一拍
//   3 拍 = 60 ns @50 MHz，对 37.5 ms 的帧周期完全可忽略；但仿真逐帧比对时要按 3 拍对齐。
//
//   ── 三个刻意的设计（也是三个坑）──────────────────────────────────────
//   ① d_eff = 0（旁路）时不能"同址读写"：简单双口 RAM 同址读写给出的是旧数据
//      （read-first），旁路会莫名慢一帧。这里让地址选"刚写进去的那一格"，
//      下一拍再读 —— 数据已经稳了，既避开读写冲突，也不用额外的旁路寄存器。
//   ② 复位屏蔽**不能**靠 RAM 里的 valid 位：BRAM/LUTRAM 上电内容是任意的（仿真里是 X），
//      写不进 0 也复位不了。这里用可复位的帧计数器 frame_cnt 判断"这一格是否已经写过"，
//      前 d_eff 帧把 found 和坐标一起清 0，未初始化内容永远进不了伺服。
//   ③ found = 0 的帧**照样要占一格、照样被延迟**：否则丢帧本身就成了另一种失真，
//      时间轴会和 A 组不等长，三张图的横轴就没法对齐了。
//
//   ── 随机源 ───────────────────────────────────────────────────────────
//   16 bit Galois LFSR（x^16+x^14+x^13+x^11+1），**每个 feat_done 推进 8 步**
//   （JIT_STEP：只推进 1 步时相邻帧高度相关，实测 lag-1 自相关 +0.38），
//   抖动模式下 d = floor(lfsr * (上限+1) / 2^16)。为什么不是 lfsr % (上限+1)：
//   变量模实测 132 LUT / 18.9 ns，过不了 cam_pclk 的 18 ns 约束，详见 jit_m 那段注释。
//   即使不用抖动模式，LFSR 也照样推进 —— 保证"上电第 N 帧用哪个随机数"只跟帧序号
//   有关，与帧间隔、之前切过哪些档都无关，A/B 来回切档时每一档都逐帧可复现。
//
//   延迟值来源（运行时可切，见 cmd_parse）：
//     D0,0  → 旁路             （回归对照：与原系统逐位等价）
//     D1,n  → 定值 n 帧        （A 组）
//     D2,n  → 抖动，上限 n 帧  （B 组，每帧独立抽签）
// -----------------------------------------------------------------------------
module delay_inject #(
    // ---------------- 参数集中在文件头 ----------------
    // D_MAX：能表达的最大帧延迟，物理深度 = D_MAX+1 格。
    //   8 帧 × 37.5 ms ≈ 300 ms，够覆盖 P5 原计划的 0/5/10/20/40 ms 好几倍。
    //   ⚠️ 指针是 4 bit，必须 D_MAX ≤ 15（深度 ≤ 16）；改大要同时加宽 wr_ptr/rd_sub。
    parameter integer D_MAX     = 8,
    // A 组默认定值档。上板时 d_depth 的真正上电值在 cmd_parse 的 D_DEPTH_DEF，
    // 这里只用于"本模块独立例化/单模块仿真"时的复位默认值（两处默认值请保持一致）。
    parameter integer D_FIXED   = 2,
    // B 组抖动上限（与 d_depth 取小）。默认 = D_MAX，即抖动可以打到最大档。
    parameter integer D_MAX_JIT = 8,
    // 16 bit Galois LFSR 种子。禁止为 0（全零是吸收态，见 SEED 的兜底）。
    parameter [15:0]  LFSR_SEED = 16'hACE1,
    // 每帧把 LFSR 推进几步。**不能是 1**，原因见下面 lfsr_adv 的注释：
    // 每帧只走 1 步时相邻帧的抽签值近似 2 倍关系，实测 lag-1 相关 +0.38、
    // 直方图第 4 档多 12%，那不是"抖动"而是一条伪随机周期序列。
    // 8 步之后低 8 位全是新 bit，实测 lag-1 ≈ 0.01。
    parameter integer JIT_STEP  = 8,
    // 上电模式：0 旁路。**默认必须是 0** —— 烧进去先和原系统逐位一样，回归不过就是接线错，
    // 而不是"延迟开着看不出来"。
    parameter [1:0]   MODE      = 2'd0
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- 来自 vision_features ----
    input  wire        feat_done,   // 每帧一个 1 拍脉冲；本拍载荷有效
    input  wire [15:0] cx_r,
    input  wire [15:0] cy_r,
    input  wire [15:0] cx_b,
    input  wire [15:0] cy_b,
    input  wire        found_r,
    input  wire        found_b,

    // ---- 来自 cmd_parse（clk_50m 域，内部 2 级同步）----
    input  wire [1:0]  d_mode,      // 0 旁路 / 1 定值 / 2 抖动
    input  wire [3:0]  d_depth,     // 定值模式的 D；抖动模式的上限

    // ---- 给 servo_polar（o_valid 接 servo 的 feat_done）----
    output reg         o_valid,
    output reg  [15:0] o_cx_r,
    output reg  [15:0] o_cy_r,
    output reg  [15:0] o_cx_b,
    output reg  [15:0] o_cy_b,
    output reg         o_found_r,
    output reg         o_found_b,

    // ---- 调试/实验记录：本帧实际施加的延迟帧数（不接也行）----
    output reg  [3:0]  o_delay
);

    // ---------------- 常量 ----------------
    localparam integer DEPTH = D_MAX + 1;             // 环缓格数

    // 参数是 integer，取位选在各家工具上宽容度不同（见 cmd_parse 里的同类注释），
    // 统一先落到固定位宽的 localparam，再参与比较/赋值。
    localparam [3:0] D_MAX_V = D_MAX;
    localparam [3:0] D_JIT_V = D_MAX_JIT;
    localparam [3:0] DEPTH_V = DEPTH;
    localparam [3:0] DFIX_V  = D_FIXED;

    // x^16 + x^14 + x^13 + x^11 + 1 的 Galois 反馈掩码。
    // 推导：x^16 ≡ x^14 + x^13 + x^11 + 1 (mod p)，所以左移溢出时异或 bit14/13/11/0。
    localparam [15:0] LFSR_POLY = 16'h6801;

    // 种子为 0 时自动兜底成 1。全零是 LFSR 的吸收态（一旦为 0 就永远是 0），
    // 表现成"抖动模式看起来像旁路"，是最难查的那种"参数写错"。
    // 注意：推进本身是双射，**非零状态永远进不了 0**，所以只在装种子处兜一次就够，
    // 每步都判 0 只是白烧两级 LUT（这条路径在 feat_done → 抽签的组合链上）。
    localparam [15:0] SEED = (LFSR_SEED != 16'd0) ? LFSR_SEED : 16'h0001;

    // 单帧推进 n 步（n 是常数参数 → 综合时展成 n 级异或链）。
    // 为什么要一次走 8 步：LFSR 每帧只走 1 步的话，第 N+1 帧的状态 ≈ 第 N 帧左移 1 位，
    // 于是 d(N+1) ≈ 2·d(N)（mod 上限），实测 lag-1 自相关 +0.38、直方图第 4 档偏高 12%，
    // 极限环会因此"看起来有规律"，B 组的结论就站不住了。走 8 步后低 8 位全是新 bit。
    // 一个 2 输入异或只要 1 个 LUT，8 步一共几十个 LUT，比"每帧走 1 步 + 假随机"划算。
    function [15:0] lfsr_adv;
        input [15:0] s;
        input integer n;
        integer k;
        reg [15:0] t;
        begin
            t = s;
            for (k = 0; k < n; k = k + 1)
                t = {t[14:0], 1'b0} ^ (t[15] ? LFSR_POLY : 16'd0);
            lfsr_adv = t;
        end
    endfunction

    // ---------------- 载荷打包 ----------------
    wire [65:0] wr_payload = {cx_r, cy_r, cx_b, cy_b, found_r, found_b};
    reg  [65:0] mem_rd;      // RAM 读口输出（1 拍）

    // ---------------- 运行配置：2 级同步 ----------------
    // d_mode/d_depth 由 clk_50m 域的 cmd_parse 驱动，跨越到 cam_pclk 域。
    // 它们一帧才可能变一次（人按键盘），2 级同步足够；万一 mode 和 depth 差一拍
    // 跨过来，最坏也只是某一帧用"新模式 + 旧深度"，延迟仍然 ≤ D_MAX，不会越界。
    reg [1:0] mode_s1, mode_s2;
    reg [3:0] dep_s1,  dep_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_s1 <= MODE;
            mode_s2 <= MODE;
            dep_s1  <= DFIX_V;
            dep_s2  <= DFIX_V;
        end else begin
            mode_s1 <= d_mode;
            mode_s2 <= mode_s1;
            dep_s1  <= d_depth;
            dep_s2  <= dep_s1;
        end
    end

    // ---------------- 延迟值计算 ----------------
    // 钳位：命令里 d_depth 能发到 15，但缓冲只有 DEPTH 格。不钳位的话读地址会绕回
    // 一个"马上要被写覆盖"的槽位，延迟实际变成负数（输出旧帧），现象极难查。
    wire [3:0] dep_lim = (dep_s2   > D_MAX_V) ? D_MAX_V : dep_s2;
    // 抖动上限再钳一次（D_MAX_JIT 可能被配得比 D_MAX 大）
    wire [3:0] jit_lim = (dep_lim  > D_JIT_V) ? D_JIT_V : dep_lim;

    // 16 bit Galois LFSR，左移型，每个 feat_done 推进 JIT_STEP 步（见上方 lfsr_adv）。
    reg  [15:0] lfsr;

    // 抖动抽签：把 16 bit LFSR 值均匀归约到 [0, jit_lim]，每帧重新抽一次，
    // 所以相邻帧的延迟互相独立 —— 这正是 B 组要的"不可预算"。
    //     d = (lfsr * (jit_lim+1)) >> 16   ≡  floor(lfsr * m / 2^16)，m = jit_lim+1
    // 为什么不是 lfsr % m（两条都是实测出来的，不是拍脑袋）：
    //   ① 变量模是借位链：实测 132 LUT / 38 级 / 18.9 ns，而 cam_pclk 的约束是 18 ns
    //      （pin.xdc: create_clock -period 18.000 [get_ports cam_pclk]）→ 必挂时序。
    //      乘-移位只占 1 个 DSP（不给 DSP 就 ~30 LUT），约 4 级 / 3 ns，余量很大。
    //   ② 分布更干净：m=9 时 lfsr%9 有 ~3.5% 偏差（2^16 mod 9 = 4），乘-移位只有 1/65536。
    //      B 组的结论是"抖动破坏极限环"，随机源本身越干净越站得住。
    // 5 bit 的 m：jit_lim=15 时 +1 会溢出成 0，除零/归零在硬件里是静默错误。
    wire [4:0]  jit_m   = {1'b0, jit_lim} + 5'd1;
    wire [20:0] jit_mul = lfsr * jit_m;
    wire [4:0]  rnd_mod5 = jit_mul[20:16];
    wire [3:0]  rnd_mod  = rnd_mod5[3:0];

    // 本帧真正要施加的延迟
    wire [3:0] d_eff = (mode_s2 == 2'd0) ? 4'd0      :   // 旁路（0 帧）
                       (mode_s2 == 2'd1) ? dep_lim   :   // A 组：定值
                                           rnd_mod;      // B 组：抖动（mode=3 兜底也走这里）

    // ---------------- 读写指针 ----------------
    reg [3:0] wr_ptr;      // 下一格要写的槽位
    // 读指针不单独存：它就是 wr_ptr - d_eff，抖动模式每帧都不一样，
    // 存一个"读指针"反而要多一条加法链，直接算偏移更省。
    wire [4:0] rd_sub  = {1'b0, wr_ptr} - {1'b0, d_eff};          // -8..+8 的 5 bit 补码
    // 深度 9 不是 2 的幂：负数时 rd_sub[3:0] 等于 x+16，再加 DEPTH 截回 4 bit 即 x+DEPTH。
    wire [3:0] rd_addr = rd_sub[4] ? (rd_sub[3:0] + DEPTH_V) : rd_sub[3:0];

    // 坑①：d_eff = 0 时 rd_addr == wr_ptr，同址读写拿到的是旧内容。改成读
    // "刚写进去的那一格"（下一拍才读，那时写已经完成），既正确又不用旁路寄存器。
    wire [3:0] sel_addr = (d_eff == 4'd0) ? wr_ptr : rd_addr;

    // ---------------- 流水线寄存器 ----------------
    // 全部先声明（Verilog-2001 不允许在后面再补声明，否则会被当成 1 bit 隐式线网）
    reg [3:0] addr_q;      // 拍1：读地址（同时也当 RAM 的读地址）
    reg [3:0] dq1, dq2;    // 拍1/拍2：本帧施加的延迟（跟着载荷一起走，o_delay 才对得上）
    reg       msk1, msk2;  // 拍1/拍2：有效屏蔽（复位后前 d 帧为 1）
    reg       v1, v2;      // 拍1/拍2：valid 流水

    // 帧计数器（可复位，见坑②）。饱和在 15：避免回绕之后"前 d 帧"的判据又成立。
    reg [3:0] frame_cnt;

    // ---------------- 环形缓冲 ----------------
    // 9 格 × 66 bit = 594 bit。实测 Vivado 2020.2 自己就把它放进 BRAM
    // （1 块 RAMB、0 LUTRAM、0 FF），所以不需要 ram_style 属性；
    // 若哪天改了 D_MAX/位宽让它退回 LUTRAM，把下面这行的注释去掉即可强制回 BRAM。
    // (* ram_style = "block" *)
    reg [65:0] mem [0:DEPTH-1];

    // 简单双口：写口 = wr_ptr，读口 = addr_q（地址已寄存，1 拍出数）。
    // 读口每拍都读：addr_q 只在 feat_done 时更新，读到的值全程稳定。
    always @(posedge clk) begin
        if (feat_done) mem[wr_ptr] <= wr_payload;
        mem_rd <= mem[addr_q];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= 4'd0;
            frame_cnt <= 4'd0;
            addr_q    <= 4'd0;
            dq1       <= 4'd0;
            dq2       <= 4'd0;
            msk1      <= 1'b1;
            msk2      <= 1'b1;
            v1        <= 1'b0;
            v2        <= 1'b0;
            o_valid   <= 1'b0;
            o_delay   <= 4'd0;
            o_cx_r    <= 16'd0;
            o_cy_r    <= 16'd0;
            o_cx_b    <= 16'd0;
            o_cy_b    <= 16'd0;
            o_found_r <= 1'b0;
            o_found_b <= 1'b0;
            lfsr      <= SEED;
        end else begin
            // ---- 拍1：俘获本帧 ----
            v1 <= feat_done;
            if (feat_done) begin
                wr_ptr <= (wr_ptr == DEPTH_V - 4'd1) ? 4'd0 : (wr_ptr + 4'd1);
                if (frame_cnt != 4'hF) frame_cnt <= frame_cnt + 4'd1;
                addr_q <= sel_addr;
                dq1    <= d_eff;
                // 这一格是否"已经写过"：帧序号 < 延迟 → 还在填缓冲 → 屏蔽
                msk1   <= (frame_cnt < d_eff);
                // LFSR 每个 feat_done 推进 JIT_STEP 步。注意：**不用抖动模式也照样推进**，
                // 这样"上电后第 N 帧用哪个随机数"只跟 N 有关，与之前切过哪些档、帧间隔多长
                // 都无关，A→B 反复切换时每一档都逐帧可复现（实验自动化最要紧的一条）。
                lfsr   <= lfsr_adv(lfsr, JIT_STEP);
            end

            // ---- 拍2：RAM 出数 ----
            dq2  <= dq1;
            msk2 <= msk1;
            v2   <= v1;

            // ---- 拍3：输出 ----
            o_valid <= v2;
            if (v2) begin
                o_delay <= dq2;
                if (msk2) begin
                    // 缓冲还没填满：整个特征帧作废（坐标一起清 0，仿真里就不会把 RAM 的 X
                    // 灌进 servo/OSD；硬件上等价于"这一帧什么都没有"）。
                    o_cx_r    <= 16'd0;
                    o_cy_r    <= 16'd0;
                    o_cx_b    <= 16'd0;
                    o_cy_b    <= 16'd0;
                    o_found_r <= 1'b0;
                    o_found_b <= 1'b0;
                end else begin
                    {o_cx_r, o_cy_r, o_cx_b, o_cy_b, o_found_r, o_found_b} <= mem_rd;
                end
            end
        end
    end

endmodule
