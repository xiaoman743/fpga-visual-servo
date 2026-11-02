#include "rt_level.h"

#include <math.h>
#include <string.h>

void rt_level_init(rt_level_t *s)
{
    memset(s, 0, sizeof(*s));
    s->sign[0] = 1.0f;
    s->sign[1] = 1.0f;
    s->arm_tps = s->wrist_tps = LEVEL_MAX_TPS;
    s->filter_ms = 80.0f;
    s->arm_window = LEVEL_ARM_WINDOW;
    s->wrist_window = LEVEL_WRIST_WINDOW;
}

/* 单轴的实际可用区间：EEPROM 硬限位（已含余量）与“固定区间 / 参考±窗口”的交集。
 * 固定模式只作用于肩/肘（i=0,1），腕关节仍按参考±窗口走。 */
static void level_limits_for(const rt_level_t *s, int i, float *lo, float *hi)
{
    *lo = s->hard_lo[i];
    *hi = s->hard_hi[i];
    if (s->fixed && i < 2) {
        *lo = fmaxf(*lo, s->fixed_lo[i]);
        *hi = fminf(*hi, s->fixed_hi[i]);
    } else {
        float window = (i == 2) ? s->wrist_window : s->arm_window;
        *lo = fmaxf(*lo, s->ref[i] - window);
        *hi = fminf(*hi, s->ref[i] + window);
    }
    if (*lo > *hi) { /* 交集为空：钉在硬限位内最接近参考的那一点，宁可不动 */
        float mid = fminf(s->hard_hi[i], fmaxf(s->hard_lo[i], s->ref[i]));
        *lo = *hi = mid;
    }
}

static void level_apply_limits(rt_level_t *s)
{
    for (int i = 0; i < 3; ++i) level_limits_for(s, i, &s->lo[i], &s->hi[i]);
}

bool rt_level_enable(rt_level_t *s, const float cmd[3],
                     const float lo[3], const float hi[3])
{
    if (s->enabled) return false; /* Do not silently recalibrate a moving arm. */
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(cmd[i]) || !isfinite(lo[i]) || !isfinite(hi[i]) ||
            lo[i] >= hi[i] || cmd[i] < lo[i] || cmd[i] > hi[i]) return false;
    }
    rt_level_t next = *s;
    for (int i = 0; i < 3; ++i) {
        next.hard_lo[i] = lo[i];
        next.hard_hi[i] = hi[i];
        next.ref[i] = next.last[i] = cmd[i];
    }
    level_apply_limits(&next);
    for (int i = 0; i < 3; ++i)
        if (cmd[i] < next.lo[i] || cmd[i] > next.hi[i]) return false; /* 固定区间不含当前姿态 */
    next.trim_deg = 0.0f;
    next.measured_ready = false;
    next.limited = false;
    next.relaxed = false;
    next.slack_deg = 0;
    next.enabled = true;
    *s = next;
    return true;
}

void rt_level_disable(rt_level_t *s)
{
    s->enabled = false;
    s->carry_mode = false;
    s->limited = false;
    s->relaxed = false;
    s->slack_deg = 0;
}

bool rt_level_set_sign(rt_level_t *s, float shoulder, float elbow)
{
    if (s->enabled || (shoulder != 1.0f && shoulder != -1.0f) ||
        (elbow != 1.0f && elbow != -1.0f)) return false;
    s->sign[0] = shoulder;
    s->sign[1] = elbow;
    return true;
}

bool rt_level_set_trim(rt_level_t *s, float degrees)
{
    if (!s->enabled || !isfinite(degrees) || fabsf(degrees) > 10.0f) return false;
    s->trim_deg = degrees; /* Absolute trim relative to the captured reference. */
    return true;
}

bool rt_level_set_source(rt_level_t *s, bool measured)
{
    if (s->enabled) return false;
    s->measured_source = measured;
    return true;
}

bool rt_level_set_speed(rt_level_t *s, float arm_tps, float wrist_tps)
{
    if (!isfinite(arm_tps) || !isfinite(wrist_tps) || arm_tps < 50 || arm_tps > 800 ||
        wrist_tps < 50 || wrist_tps > 1200) return false;
    s->arm_tps = arm_tps;
    s->wrist_tps = wrist_tps;
    return true;
}

bool rt_level_set_filter(rt_level_t *s, float milliseconds)
{
    if (!isfinite(milliseconds) || milliseconds < 10 || milliseconds > 200) return false;
    s->filter_ms = milliseconds;
    return true;
}

bool rt_level_set_tolerance(rt_level_t *s, float degrees)
{
    if (!isfinite(degrees) || degrees < 0 || degrees > 10 ||
        (s->enabled && fabsf(s->slack_deg) > degrees)) return false;
    s->tolerance_deg = degrees;
    return true;
}

bool rt_level_set_range(rt_level_t *s, float arm_ticks, float wrist_ticks)
{
    if (!isfinite(arm_ticks) || !isfinite(wrist_ticks) || arm_ticks < 50 || arm_ticks > 1200 ||
        wrist_ticks < 50 || wrist_ticks > 1600) return false;
    rt_level_t next = *s;
    next.arm_window = arm_ticks;
    next.wrist_window = wrist_ticks;
    if (next.enabled) {
        level_apply_limits(&next);
        for (int i = 0; i < 3; ++i)
            if (next.last[i] < next.lo[i] || next.last[i] > next.hi[i]) return false;
    }
    *s = next;
    return true;
}

bool rt_level_set_fixed(rt_level_t *s, bool on, const float lo[2], const float hi[2])
{
    if (!s) return false;
    if (on && (!lo || !hi || !isfinite(lo[0]) || !isfinite(hi[0]) ||
               !isfinite(lo[1]) || !isfinite(hi[1]) || lo[0] >= hi[0] || lo[1] >= hi[1]))
        return false;
    rt_level_t next = *s;
    next.fixed = on;
    if (on) {
        for (int i = 0; i < 2; ++i) { next.fixed_lo[i] = lo[i]; next.fixed_hi[i] = hi[i]; }
    }
    if (next.enabled) {
        level_apply_limits(&next);
        for (int i = 0; i < 3; ++i)
            if (next.last[i] < next.lo[i] || next.last[i] > next.hi[i]) return false;
    }
    *s = next;
    return true;
}

bool rt_level_seed_measurement(rt_level_t *s, const float measured[2])
{
    for (int i = 0; i < 2; ++i)
        if (!isfinite(measured[i]) || measured[i] < 0 || measured[i] > 4095) return false;
    for (int i = 0; i < 2; ++i)
        s->measured_ref[i] = s->measured_filtered[i] = measured[i];
    s->measured_ready = true;
    return true;
}

bool rt_level_update_measurement(rt_level_t *s, const float measured[2], float dt_ms)
{
    if (!s->measured_ready || !isfinite(dt_ms) || dt_ms <= 0) return false;
    /* Configurable low-pass, reject a corrupt sample instead of commanding a wrist jump.
     * At the 200 tick/s coupled speed, 120 ticks is far above one read interval. */
    for (int i = 0; i < 2; ++i)
        if (!isfinite(measured[i]) || measured[i] < 0 || measured[i] > 4095 ||
            fabsf(measured[i] - s->measured_filtered[i]) > 120.0f) return false;
    float dt = fminf(dt_ms, 10.0f);
    float alpha = dt / (s->filter_ms + dt);
    for (int i = 0; i < 2; ++i)
        s->measured_filtered[i] += alpha * (measured[i] - s->measured_filtered[i]);
    return true;
}

void rt_level_apply(rt_level_t *s, float cmd[3], float dt_ms, bool allow_motion)
{
    if (!s->enabled) return;
    s->limited = false;
    if (!allow_motion || !isfinite(dt_ms) || dt_ms <= 0.0f ||
        (s->measured_source && !s->measured_ready)) {
        memcpy(cmd, s->last, sizeof(s->last));
        return;
    }

    float want[3] = {cmd[0], cmd[1], 0.0f};
    want[2] = s->ref[2] - s->sign[0] * (want[0] - s->ref[0])
              - s->sign[1] * (want[1] - s->ref[1])
              + s->trim_deg * LEVEL_TICKS_PER_DEG;
    if (s->measured_source)
        want[2] = s->ref[2] - s->sign[0] * (s->measured_filtered[0] - s->measured_ref[0])
                  - s->sign[1] * (s->measured_filtered[1] - s->measured_ref[1])
                  + s->trim_deg * LEVEL_TICKS_PER_DEG;
    bool measured_escape = false;
    if (s->measured_source && !s->carry_mode) {
        /* Feedback can drift beyond the pitch allowance after a previously
         * accepted step (load offset, backlash, filter delay). Rejecting all
         * targets there also rejects the reverse command needed to escape.
         * Check the requested arm delta: inside the allowance stay inside;
         * outside permit only a strictly improving step, never one crossing
         * into the opposite forbidden side. Still track the measured wrist
         * target below; do not replace measured compensation with commands. */
        float projected = want[2] - s->sign[0] * (want[0] - s->last[0])
                                 - s->sign[1] * (want[1] - s->last[1]);
        float allowance = s->tolerance_deg * LEVEL_TICKS_PER_DEG;
        float low = s->lo[2] - allowance, high = s->hi[2] + allowance;
        bool permitted = isfinite(want[2]) && isfinite(projected);
        if (want[2] < low) {
            measured_escape = projected > want[2] && projected <= high;
            permitted = permitted && measured_escape;
        } else if (want[2] > high) {
            measured_escape = projected < want[2] && projected >= low;
            permitted = permitted && measured_escape;
        } else {
            permitted = permitted && projected >= low && projected <= high;
        }
        if (!permitted) {
            s->limited = true;
            memcpy(cmd, s->last, sizeof(s->last));
            return;
        }
    }
    /* Approach uses the configured pitch tolerance (or a reducing escape).
     * Carrying keeps the nearest legal wrist target without letting wrist
     * saturation block the arm. Arm boundaries never relax. */
    bool target_relaxed = false;
    float target_slack = 0;
    if (isfinite(want[2]) && (want[2] < s->lo[2] || want[2] > s->hi[2])) {
        float wrist = fminf(s->hi[2], fmaxf(s->lo[2], want[2]));
        float slack = (wrist - want[2]) / LEVEL_TICKS_PER_DEG;
        if (fabsf(slack) <= s->tolerance_deg || measured_escape || s->carry_mode) {
            want[2] = wrist;
            target_relaxed = true;
            target_slack = slack;
        }
    }
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(want[i]) || want[i] < s->lo[i] || want[i] > s->hi[i]) {
            s->limited = true;
            memcpy(cmd, s->last, sizeof(s->last));
            return;
        }
    }

    /* Do not release a backlog as a jump after a long control-loop pause. */
    float dt = fminf(dt_ms, 10.0f) / 1000.0f;
    float scale = 1.0f;
    for (int i = 0; i < 3; ++i) {
        float distance = fabsf(want[i] - s->last[i]);
        float step = (i == 2 ? s->wrist_tps : s->arm_tps) * dt;
        if (distance > step) scale = fminf(scale, step / distance);
    }
    for (int i = 0; i < 3; ++i) {
        cmd[i] = s->last[i] + scale * (want[i] - s->last[i]);
        s->last[i] = cmd[i];
    }
    s->relaxed = target_relaxed;
    s->slack_deg = target_slack;
}
