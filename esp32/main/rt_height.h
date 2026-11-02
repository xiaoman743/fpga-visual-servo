#ifndef RT_HEIGHT_H
#define RT_HEIGHT_H

#include <stdbool.h>

/* Heights are for one fixed tool point above the table, in millimetres.
 * The table origin includes shoulder height and the fixed vertical tool offset.
 * Geometry must be calibrated; the historic visual FK offsets are not defaults.
 * The approach ceiling assumes the level controller holds tool orientation;
 * carry mode suspends that ceiling while preserving the original reference. */
typedef struct {
    float off2, off3, origin_mm, sign;
} rt_height_model_t;

typedef enum {
    HEIGHT_REQ_OFF, HEIGHT_REQ_ON, HEIGHT_REQ_STAT, HEIGHT_REQ_MODEL,
    HEIGHT_REQ_REF, HEIGHT_REQ_LIMIT
} rt_height_action_t;

typedef struct {
    rt_height_action_t action;
    float a, b, c, d;
} rt_height_request_t;

typedef struct {
    bool model_ready, referenced, enabled, limited, fault;
    /* A grasp close command suspends only the geometric ceiling. Keep
     * feedback checks, references and accepted-command tracking active. */
    bool carry_mode;
    rt_height_model_t model;
    float limit_mm, margin_mm;
    float cmd_bias[2], last[2];
    float measured_mm, predicted_mm;
} rt_height_t;

void rt_height_init(rt_height_t *s);
bool rt_height_set_model(rt_height_t *s, float off2, float off3, float sign);
bool rt_height_set_limit(rt_height_t *s, float limit_mm, float margin_mm);
bool rt_height_forward(const rt_height_model_t *model, const float q[2], float *height);
/* Reference changes no joint. It anchors the SAME fixed tool point to the table. */
bool rt_height_reference(rt_height_t *s, const float measured[2], float height_mm);
bool rt_height_enable(rt_height_t *s, const float cmd[2], const float measured[2]);
void rt_height_disable(rt_height_t *s);
/* Filter the proposed shoulder/elbow target BEFORE level compensation.
 * The full proposed path is checked, not only the end point. Descending escapes
 * remain possible when feedback has drifted above the threshold.
 * Commit only the targets actually accepted by the downstream level controller. */
void rt_height_apply(rt_height_t *s, float cmd[2], const float measured[2], bool healthy);
void rt_height_commit(rt_height_t *s, const float accepted[2]);

#endif
