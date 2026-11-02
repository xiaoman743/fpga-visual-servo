/*
 * fpga_kin.h -- 两连杆正运动学 + 径向反解（只服务 CH2/CH3）
 * ============================================================================
 * 为什么需要（2026-10-28 结论）：
 *   之前把图像误差分别乘 0.5 推给肩/肘，是"瞎推"：两个关节都会同时改变
 *   径向 r 和高度 h，于是"伸出去"变成"往上飘"。实测：reach 累计 1206 tick，
 *   画面半径只 456->470，而臂被顶到了桌面。
 *
 *   有了正运动学就能把两者分开：
 *     r = L1·cosθ1 + L2·cos(θ1+θ2)     水平距离
 *     h = L1·sinθ1 + L2·sin(θ1+θ2)     高度
 *   要给"只伸 Δr、高度不变"，就解 2x2 雅可比求 (Δθ1, Δθ2)。
 *
 * ⚠️ 底座 CH1 不在这里：它绕竖直轴转，只改图像极角 θ，不改 r/h。
 *    两个回路天然解耦（这也是为什么 pan 能单独调好）。
 *
 * 标定来源（2026-10-28 实测，见 _标定-关节零位记录.md）：
 *   L1 = 115 mm (CH2->CH3)      L2 = 136 mm (CH3->CH4)
 *   offset2 = 3043 (θ1 = 0 时上臂水平)   offset3 = 1011 (θ2 = 0 时肘伸直)
 *   s2 = +1, s3 = +1（j2/j3 增大都对应角度增大）
 *   ⚠️ 这些都是"当前这台臂 + 当前装夹"的数，换臂要重标。
 * ========================================================================== */
#ifndef FPGA_KIN_H
#define FPGA_KIN_H

#include <stdbool.h>
#include <stdint.h>

/* ---- 机械参数（标定值） ---- */
#define KIN_L1_MM 115.0f
#define KIN_L2_MM 136.0f
#define KIN_OFF2 3043.0f /* j2 在 θ1 = 0 时的 tick */
#define KIN_OFF3 1011.0f /* j3 在 θ2 = 0 时的 tick */
#define KIN_S2 (+1.0f)
#define KIN_S3 (+1.0f)
#define KIN_DEG_PER_TICK (360.0f / 4096.0f) /* ≈0.0879 */

/* ---- 安全边界（角度解算用，不是机械限位） ---- */
#define KIN_DTH_MAX_DEG 3.0f   /* 单帧角度增量上限（≈ 全速 2°/帧 的 1.5 倍） */
#define KIN_DQ_MAX_TICK 40.0f  /* 单帧关节 tick 上限（冗余保护） */
#define KIN_DET_EPS 1.0e-3f    /* 雅可比行列式下限：太小说明臂伸直了，拒绝解算 */

typedef struct {
    float th1_deg; /* 肩相对水平的角（上抬为正） */
    float th2_deg; /* 肘相对上臂的角（伸直为 0） */
    float r_mm;    /* 末端(CH4 轴)到基座轴的水平距离 */
    float h_mm;    /* 末端相对肩轴的高度 */
} kin_pose_t;

/* 正解：由 j2/j3 算 (r, h)。失败（角度超界）返回 false。 */
bool kin_forward(int16_t j2, int16_t j3, kin_pose_t *out);

/* 反解：在当前位姿下，要让末端"径向变化 dr_mm 且高度变化 dh_mm"，
 * 肩/肘各应转多少 tick。成功返回 true 并写 dq2/dq3（可为负）。
 * 奇异（臂伸直/折回）或增量过大时返回 false —— 调用方应保持不动。 */
bool kin_solve_reach(int16_t j2, int16_t j3, float dr_mm, float dh_mm, float *dq2,
                     float *dq3);

#endif /* FPGA_KIN_H */
