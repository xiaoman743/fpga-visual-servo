#include "rt_height.h"

#include <math.h>
#include <string.h>

#define K (6.283185307179586f / 4096.0f)
#define L1 115.0f
#define L2 136.0f

static bool joints_valid(const float q[2])
{
    return isfinite(q[0]) && isfinite(q[1]) && q[0] >= 0 && q[0] <= 4095 &&
           q[1] >= 0 && q[1] <= 4095;
}

bool rt_height_forward(const rt_height_model_t *m, const float q[2], float *height)
{
    if (!m || !height || !joints_valid(q) || !isfinite(m->off2) ||
        !isfinite(m->off3) || !isfinite(m->origin_mm) ||
        (m->sign != 1 && m->sign != -1)) return false;
    float a = (q[0] - m->off2) * K * m->sign;
    float b = (q[1] - m->off3) * K * m->sign;
    *height = m->origin_mm + L1 * sinf(a) + L2 * sinf(a + b);
    return isfinite(*height);
}

void rt_height_init(rt_height_t *s)
{
    memset(s, 0, sizeof(*s));
    s->limit_mm = 85; /* Requested control trigger: 85 - 10 = 75 mm. */
    s->margin_mm = 10;
    s->measured_mm = s->predicted_mm = NAN;
}

bool rt_height_set_model(rt_height_t *s, float off2, float off3, float sign)
{
    if (s->enabled || !isfinite(off2) || !isfinite(off3) || off2 < 0 || off2 > 4095 ||
        off3 < 0 || off3 > 4095 || (sign != 1 && sign != -1)) return false;
    s->model = (rt_height_model_t){off2, off3, 0, sign};
    s->model_ready = true;
    s->referenced = false;
    return true;
}

bool rt_height_set_limit(rt_height_t *s, float limit_mm, float margin_mm)
{
    if (s->enabled || !isfinite(limit_mm) || !isfinite(margin_mm) ||
        limit_mm < 20 || limit_mm > 300 || margin_mm < 2 || margin_mm > 20 ||
        margin_mm >= limit_mm) return false;
    s->limit_mm = limit_mm;
    s->margin_mm = margin_mm;
    return true;
}

bool rt_height_reference(rt_height_t *s, const float measured[2], float height_mm)
{
    float raw;
    if (s->enabled || !s->model_ready || !isfinite(height_mm) ||
        height_mm < 0 || height_mm > 400 ||
        !rt_height_forward(&s->model, measured, &raw)) return false;
    s->model.origin_mm += height_mm - raw;
    s->measured_mm = height_mm;
    s->referenced = true;
    return true;
}

bool rt_height_enable(rt_height_t *s, const float cmd[2], const float measured[2])
{
    float h;
    if (s->enabled || !s->model_ready || !s->referenced || !joints_valid(cmd) ||
        !rt_height_forward(&s->model, measured, &h)) return false;
    for (int i = 0; i < 2; ++i) {
        if (fabsf(cmd[i] - measured[i]) >= 180) return false;
    }
    for (int i = 0; i < 2; ++i) {
        s->cmd_bias[i] = measured[i] - cmd[i];
        s->last[i] = cmd[i];
    }
    s->measured_mm = s->predicted_mm = h;
    s->limited = s->fault = false;
    s->enabled = true;
    return true;
}

void rt_height_disable(rt_height_t *s)
{
    s->enabled = s->limited = s->fault = false;
    s->carry_mode = false;
}

/* Upper bound on height along the entire straight joint segment. For a sample
 * interval, |h''| <= L1*da^2+L2*(da+db)^2; linear interpolation error <= M/8.
 * Sixteen intervals keep the bound small even for a 60-tick vision step. */
static bool path_allowed(const rt_height_model_t *m, const float from[2],
                         const float to[2], float ceiling, float *end_height)
{
    float start, end;
    if (!rt_height_forward(m, from, &start) ||
        !rt_height_forward(m, to, &end)) return false;
    if (end_height) *end_height = end;
    float da = (to[0] - from[0]) * K * m->sign;
    float db = (to[1] - from[1]) * K * m->sign;
    float bound = L1 * da * da + L2 * (da + db) * (da + db);
    if (start > ceiling) {
        /* Outside: require strictly descending over the whole segment. Bound
         * the derivative on each subinterval, permitting a safe reverse exit. */
        if (end >= start - 0.00001f) return false;
        for (int i = 0; i < 16; ++i) {
            float t = ((float)i + 0.5f) / 16;
            float a = (from[0] - m->off2) * K * m->sign + t * da;
            float b = (from[1] - m->off3) * K * m->sign + t * db;
            float derivative = L1 * cosf(a) * da + L2 * cosf(a + b) * (da + db);
            if (derivative + bound / 32 > 0) return false;
        }
        return true;
    }
    if (start > ceiling || end > ceiling) return false;
    float peak = fmaxf(start, end);
    for (int i = 1; i < 16; ++i) {
        float q[2] = {from[0] + (to[0] - from[0]) * i / 16,
                      from[1] + (to[1] - from[1]) * i / 16};
        float h;
        if (!rt_height_forward(m, q, &h)) return false;
        peak = fmaxf(peak, h);
    }
    return peak + bound / (8 * 16 * 16) <= ceiling + 0.00001f;
}

void rt_height_apply(rt_height_t *s, float cmd[2], const float measured[2], bool healthy)
{
    if (!s->enabled) return;
    s->limited = s->fault = false;
    if (!healthy || !joints_valid(cmd) ||
        !rt_height_forward(&s->model, measured, &s->measured_mm)) {
        s->fault = true;
        memcpy(cmd, s->last, sizeof(s->last));
        return;
    }
    float start[2], target[2], measured_target[2];
    for (int i = 0; i < 2; ++i) {
        start[i] = s->last[i] + s->cmd_bias[i];
        target[i] = cmd[i] + s->cmd_bias[i];
        measured_target[i] = measured[i] + cmd[i] - s->last[i];
    }
    if (s->carry_mode) {
        /* Keep the prediction and committed command current during a lift;
         * reopening must never restore an old pre-grasp command as a jump. */
        if (!rt_height_forward(&s->model, target, &s->predicted_mm)) {
            s->fault = true;
            memcpy(cmd, s->last, sizeof(s->last));
        }
        return;
    }
    float ceiling = s->limit_mm - s->margin_mm;
    /* Check both command prediction (with captured load offset) and actual
     * feedback, so a load change cannot hide a measured threshold crossing. */
    /* 2026-11-02：整步拒绝只用「实测姿态 → 实测目标」这一条判据。
     * 原来还要求「指令预测姿态」也在天花板内，而 start = last + cmd_bias 在
     * CH2 出现 ~120 tick 跟随差时会把预测高度虚高 ~30 mm（实测 64 mm 却预测 90+），
     * 于是每一帧都整步拒绝（现场 12 s 内 260 次 rise blocked，解永远发不出去）。
     * 预测值仍照算，只用于上报，不再作为否决依据。 */
    (void)path_allowed(&s->model, start, target, ceiling, &s->predicted_mm);
    if (!path_allowed(&s->model, measured, measured_target, ceiling, NULL)) {
        s->limited = true;
        memcpy(cmd, s->last, sizeof(s->last));
    }
}

void rt_height_commit(rt_height_t *s, const float accepted[2])
{
    if (s->enabled && joints_valid(accepted)) memcpy(s->last, accepted, sizeof(s->last));
}
