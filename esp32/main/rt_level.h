#ifndef RT_LEVEL_H
#define RT_LEVEL_H

#include <stdbool.h>

/* All values are servo command ticks, ordered shoulder/elbow/wrist pitch.
 * Signs are d(physical pitch)/d(tick), relative to the wrist sign.
 * Default +/+ is inferred from the two horizontal snapshots on 2026-10-28;
 * a small-motion physical check is still required. */
#define LEVEL_TICKS_PER_DEG (4096.0f / 360.0f)
#define LEVEL_MAX_TPS 200.0f
#define LEVEL_ARM_WINDOW 200.0f
#define LEVEL_WRIST_WINDOW 400.0f

typedef enum {
    LEVEL_REQ_ON, LEVEL_REQ_OFF, LEVEL_REQ_STAT, LEVEL_REQ_TRIM, LEVEL_REQ_SIGN,
    LEVEL_REQ_SOURCE, LEVEL_REQ_SPEED, LEVEL_REQ_FILTER, LEVEL_REQ_RANGE,
    LEVEL_REQ_TOLERANCE
} rt_level_action_t;

typedef struct {
    rt_level_action_t action;
    float a, b;
} rt_level_request_t;

typedef struct {
    bool enabled;
    bool limited;
    bool relaxed;
    /* Carrying: track horizontal where possible, but do not stop the arm
     * for wrist pitch slack. Joint windows and feedback guards still apply. */
    bool carry_mode;
    float tolerance_deg, slack_deg;
    float sign[2];
    float ref[3];
    float last[3];
    float lo[3], hi[3];
    float trim_deg;
    bool measured_source;
    bool measured_ready;
    float measured_ref[2];
    float measured_filtered[2];
    float arm_tps, wrist_tps, filter_ms;
    float arm_window, wrist_window;
    /* 固定工作区间：on 时肩/肘不再用“参考位置 ± 窗口”，直接用 fixed_lo/hi，
     * 但仍与 hard_lo/hard_hi（EEPROM 余量）取交集，硬限位永远是最后一道。 */
    bool fixed;
    float fixed_lo[2], fixed_hi[2];
    float hard_lo[3], hard_hi[3];
} rt_level_t;

void rt_level_init(rt_level_t *s);
/* Enable at a physically horizontal, stationary pose. No command is changed.
 * Capturing COMMAND references retains existing command-measure load offsets. */
bool rt_level_enable(rt_level_t *s, const float cmd[3],
                     const float lo[3], const float hi[3]);
void rt_level_disable(rt_level_t *s);
bool rt_level_set_sign(rt_level_t *s, float shoulder, float elbow);
bool rt_level_set_trim(rt_level_t *s, float degrees);
bool rt_level_set_source(rt_level_t *s, bool measured);
bool rt_level_set_speed(rt_level_t *s, float arm_tps, float wrist_tps);
bool rt_level_set_filter(rt_level_t *s, float milliseconds);
/* At a wrist bound only, accept up to this modeled pitch deviation (0..10deg).
 * In the interior the wrist continues targeting the horizontal reference. */
bool rt_level_set_tolerance(rt_level_t *s, float degrees);
/* Expansion never moves a joint or recaptures the horizontal reference.
 * A shrink excluding any current accepted command is refused. */
bool rt_level_set_range(rt_level_t *s, float arm_ticks, float wrist_ticks);
/* 固定肩/肘工作区间（CH2、CH3，顺序与 lo[0..1] 一致）。开启后不用开机窗口；
 * 传入值仍会被 EEPROM 余量收窄，超出部分只在该硬限位内生效。 */
bool rt_level_set_fixed(rt_level_t *s, bool on, const float lo[2], const float hi[2]);
bool rt_level_seed_measurement(rt_level_t *s, const float shoulder_elbow[2]);
bool rt_level_update_measurement(rt_level_t *s, const float shoulder_elbow[2], float dt_ms);
/* Run after command generation and before the bus write. With allow_motion=false
 * hold the previous accepted targets, including any unfulfilled trim.
 * Reject out-of-range coupled moves, rather than clipping the wrist alone.
 * If measured pitch drifts outside the allowance, permit only arm deltas
 * reducing that violation, so feedback-based gating cannot trap a reversal.
 * Scale all three deltas together to respect the compensation speed limit. */
void rt_level_apply(rt_level_t *s, float cmd[3], float dt_ms, bool allow_motion);

#endif
