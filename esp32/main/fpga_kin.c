/*
 * fpga_kin.c -- 两连杆正解 + 径向反解实现
 * ============================================================================
 * 全部是纯数学：不持有状态、不碰外设，方便单测和现场复核。
 *
 * 反解公式（J [dθ1 dθ2]^T = [dr dh]^T，2x2 直接求逆）：
 *   dr/dθ1 = -L1 sinθ1 - L2 sin(θ1+θ2)
 *   dr/dθ2 = -L2 sin(θ1+θ2)
 *   dh/dθ1 =  L1 cosθ1 + L2 cos(θ1+θ2)
 *   dh/dθ2 =  L2 cos(θ1+θ2)
 *   det    = dr/dθ1·dh/dθ2 - dr/dθ2·dh/dθ1
 *   dθ1 = ( dh/dθ2·dr - dr/dθ2·dh) / det
 *   dθ2 = (-dh/dθ1·dr + dr/dθ1·dh) / det
 * 角度单位 rad，长度 mm。
 *
 * 已知奇异点：θ2 ≈ 0（完全伸直）和 θ2 ≈ ±180°（完全折回）时 det → 0。
 * 实测这台臂在“全水平伸直”位姿就接近 θ2=0，所以 det 保护是必须的。
 * ========================================================================== */
#include "fpga_kin.h"

#include <math.h>

#define DEG2RAD (3.14159265358979f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979f)

static float clampf_local(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* j2/j3 -> θ1/θ2（度） */
static void joints_to_angles(int16_t j2, int16_t j3, float *th1_deg, float *th2_deg)
{
    *th1_deg = ((float)j2 - KIN_OFF2) * KIN_DEG_PER_TICK * KIN_S2;
    *th2_deg = ((float)j3 - KIN_OFF3) * KIN_DEG_PER_TICK * KIN_S3;
}

bool kin_forward(int16_t j2, int16_t j3, kin_pose_t *out)
{
    if (out == NULL) {
        return false;
    }
    float th1, th2;
    joints_to_angles(j2, j3, &th1, &th2);
    if (th1 < -170.0f || th1 > 170.0f || th2 < -170.0f || th2 > 170.0f) {
        return false;
    }
    float a1 = th1 * DEG2RAD;
    float a12 = (th1 + th2) * DEG2RAD;

    out->th1_deg = th1;
    out->th2_deg = th2;
    out->r_mm = KIN_L1_MM * cosf(a1) + KIN_L2_MM * cosf(a12);
    out->h_mm = KIN_L1_MM * sinf(a1) + KIN_L2_MM * sinf(a12);
    return true;
}

bool kin_solve_reach(int16_t j2, int16_t j3, float dr_mm, float dh_mm, float *dq2,
                     float *dq3)
{
    if (dq2 == NULL || dq3 == NULL) {
        return false;
    }

    float th1, th2;
    joints_to_angles(j2, j3, &th1, &th2);
    float a1 = th1 * DEG2RAD;
    float a12 = (th1 + th2) * DEG2RAD;

    float s1 = sinf(a1), c1 = cosf(a1);
    float s12 = sinf(a12), c12 = cosf(a12);

    float drd1 = -KIN_L1_MM * s1 - KIN_L2_MM * s12;
    float drd2 = -KIN_L2_MM * s12;
    float dhd1 = KIN_L1_MM * c1 + KIN_L2_MM * c12;
    float dhd2 = KIN_L2_MM * c12;

    float det = drd1 * dhd2 - drd2 * dhd1;
    if (fabsf(det) < KIN_DET_EPS) {
        return false; /* 奇异：臂伸直/折回，这一帧不伸 */
    }

    /* 解 J·dθ = [dr, dh]，角度用弧度 */
    float dth1 = (dhd2 * dr_mm - drd2 * dh_mm) / det;
    float dth2 = (-dhd1 * dr_mm + drd1 * dh_mm) / det;

    float dth1_deg = dth1 * RAD2DEG;
    float dth2_deg = dth2 * RAD2DEG;

    /* 单帧角度上限：超了就整体等比缩，保持方向不变（不要裁某一轴，会走歪） */
    float amax = fmaxf(fabsf(dth1_deg), fabsf(dth2_deg));
    if (amax > KIN_DTH_MAX_DEG) {
        float k = KIN_DTH_MAX_DEG / amax;
        dth1_deg *= k;
        dth2_deg *= k;
    }

    /* 角度 -> 关节 tick（除以 deg/tick 和符号） */
    float q2 = dth1_deg / (KIN_DEG_PER_TICK * KIN_S2);
    float q3 = dth2_deg / (KIN_DEG_PER_TICK * KIN_S3);

    q2 = clampf_local(q2, -KIN_DQ_MAX_TICK, KIN_DQ_MAX_TICK);
    q3 = clampf_local(q3, -KIN_DQ_MAX_TICK, KIN_DQ_MAX_TICK);

    *dq2 = q2;
    *dq3 = q3;
    return true;
}
