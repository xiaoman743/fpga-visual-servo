/*
 * fpga_pos.h -- FPGA 视觉伺服 · **位置式**输出（替代速率式）
 * ============================================================================
 * 为什么必须换成位置式（2026-10-28 实测结论）：
 *
 *   速率式： 位置 = 位置 + 速率 × 经过时间
 *                          ^^^^^^^^^^^^ 这一项全是误差源：
 *                          · FPGA 帧率抖动（24~27 fps）
 *                          · 坏帧 19~25%，丢的帧仍然在"走"
 *                          · stale 回落时最后一条速度还会再跑 100 ms
 *           实测后果：半径 437->503->202 一路甩过头、臂"向上飞"到软限位。
 *
 *   位置式： 目标 += 每帧增量
 *                          ^^^^^^^^^^ 与墙上时间无关，帧数就是节拍：
 *                          · 丢帧 => 目标不动（安全，天然 fail-safe）
 *                          · 不需要 dt，不需要积分，不会漂移
 *                          · 限位是**位置窗口**（相对使能时的姿态），不是"走多少算多少"
 *
 * 单位约定：
 *   FPGA 1 tick ≈ 0.5°；ESP32 舵机 1 tick = 360°/4096。换算系数 TICK_TO_SERVO。
 *   极性来自 jog 实测（见 交接-20261028 §8.2）：pan / shoulder / elbow 全部取负。
 *   ⚠️ 极性属安装几何，换相机/换臂要重新用 `fpga jog` 复核。
 * ========================================================================== */
#ifndef FPGA_POS_H
#define FPGA_POS_H

#include <stdbool.h>
#include <stdint.h>
#include "vision_frame.h"

#define FPGA_POS_NJ 6 /* 关节数 */

/* ---- 可调参数（全部集中在这里）---- */
#define FPGA_POS_TICK_TO_SERVO (4096.0f / 360.0f / 2.0f) /* ≈5.6889 */

/* 每帧增量 -> 舵机 tick 的增益。1.0 表示 FPGA 说走 1 tick(0.5°)，关节就走 0.5°。 */
#define FPGA_POS_G_PAN 1.0f
#define FPGA_POS_G_SHOULDER 0.5f
#define FPGA_POS_G_ELBOW 0.5f

/* 底座死区：横向误差小于它就不动 CH1。
 * 现场问题：伸到最后几 px 时底座还在按 cross 每帧蹭 ±2 tick，臂被来回带偏；
 * 15 px 与 ALIGN 的横向容差一致，进入死区后只由肩肘做径向。 */
#define FPGA_POS_PAN_DEADBAND_PX 15.0f

/* ★ 抗积分饱和：目标最多领先**实测位置**这么多（舵机 tick，150 ≈ 13°）。
 * 没有这一条时实测过：目标累加得比机械臂跑得快，目标先跑到窗口边，
 * 臂追上来必然冲过头 -> 角差从 7.8° 一路涨到 21.9°（振荡发散）。
 * 有了它，目标永远只比臂"领先一点点"，FPGA 的 P 会自然降下来。
 * 2026-11-03：60 → 150。现场反复出现"目标被钉在 measured−60、关节不跟、
 * 且不报错"的静默死锁（跟随差恒为 −60.0 就是窗口边界）。放宽窗口让目标
 * 有推进余量；配合 vision_auto 里新增的 stall_ms 检测，真卡住会明确 ABORT。 */
#define FPGA_POS_LEAD_TICKS 150.0f

/* ---- 径向：图像 -> 毫米 -> 关节（FK/IK）----
 * 极心 c 与 FPGA 里的一致；k = px/mm 由"全水平伸直 600px ≈ 251mm"标定。 */
#define FPGA_POS_CX 409.0f
#define FPGA_POS_CY (-165.0f)
#define FPGA_POS_K_PX_PER_MM 2.30f

/* 每帧只走剩余误差的这个比例，且单帧不超过 STEP_MM —— 相当于 P 控制，
 * 太快会像之前一样冲过头。 */
#define FPGA_POS_REACH_ALPHA 0.35f
#define FPGA_POS_REACH_STEP_MM 5.0f

/* 单帧目标最大位移（舵机 tick）。FPGA 的 |P|/|E| ≤ 8，8×5.69≈45，
 * 这里留到 60 作兜底，防止某帧脏数据把目标一步推飞。 */
#define FPGA_POS_STEP_MAX 60.0f

/* 目标限制在舵机合法区间内（STS3215 = 0..4095，留 20 tick 余量）。 */
#define FPGA_POS_MIN 20.0f
#define FPGA_POS_MAX 4075.0f

/* ---- 接口 ---------------------------------------------------------------- */

/* 清状态（不使能）。 */
void fpga_pos_reset(void);

/* 以"当前实测姿态"为基准/窗口中心，并把目标设成当前姿态（上电不跳变）。
 * meas_pos 为 6 个关节的实测舵机 tick；越界值会被夹到合法区间。 */
void fpga_pos_arm(const int16_t meas_pos[FPGA_POS_NJ]);

/* 每收到**一帧有效** FPGA 数据调用一次。
 * pan_inc 用 FPGA 的 P；reach 走 FK/IK（用 rx/ry/bx/by 算图像半径误差）。 */
void fpga_pos_step_frame(const fpga_frame_t *frame);
void fpga_pos_service(void);
bool fpga_pos_axis_owned(unsigned axis);
void fpga_pos_arm_at_command(const int16_t meas[6],const float cmd[6]);
void fpga_pos_commit(const float accepted[3]);
void fpga_pos_auto_gates(bool enabled,bool pan,bool reach);
bool fpga_pos_map_ready(void);
bool fpga_pos_get_float(float out[6]);
void fpga_pos_observe_frame(const fpga_frame_t *frame);
void fpga_pos_set_height_model(bool ready,float off2,float off3,float sign);
void fpga_pos_clear_samples(void);
bool fpga_pos_add_sample(float q1,float q2,float q3,float dr2,float dr3);
unsigned fpga_pos_sample_count(void);
void fpga_pos_track_frame(const fpga_frame_t *frame);
unsigned fpga_pos_online_count(void);

/* 取当前目标位置（舵机 tick）。未 arm 时返回 false。 */
bool fpga_pos_get(int16_t out[FPGA_POS_NJ]);

/* 位置式是否在用（main 循环据此决定走哪条路径）。 */
bool fpga_pos_active(void);

/* 模式已选（无论是否 arm 完成）。主循环据此决定走位置式还是速率式。 */
bool fpga_pos_selected(void);

/* 运行时可调增益（百分比，100 = 用编译期 FPGA_POS_G_*，0 = 关掉该轴）。
 * 与 pga gain 命令共用同一组数，位置式与速率式行为一致。 */
void fpga_pos_set_gain_pct(int pan_pct, int sh_pct, int el_pct);
void fpga_pos_set_active(bool on);

/* 主循环把实测姿态递给本模块用（arm 时需要）。 */
/* 每周期把实测位置递进来（抗饱和用）。 */
void fpga_pos_set_meas(const int16_t meas_pos[FPGA_POS_NJ]);

/* 把水平约束当前允许的 CH2/CH3 区间递进来：一个轴顶到边界时，
 * 径向需求会自动转给另一个轴，而不是整段卡住。 */
void fpga_pos_set_limits(float lo2, float hi2, float lo3, float hi3);

bool fpga_pos_needs_arm(void);
void fpga_pos_note_armed(void);

#endif /* FPGA_POS_H */
