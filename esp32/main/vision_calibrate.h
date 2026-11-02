#ifndef VISION_CALIBRATE_H
#define VISION_CALIBRATE_H
#include <stdbool.h>
#include "vision_frame.h"
typedef struct {
    bool active, failed, done, initialized, pending;
    unsigned phase, count, corrections, elapsed_ms, pause_ms, pending_ms;
    unsigned late_cycles; /* 连续超时周期；单次卡顿只冻结，不再直接判失败 */
    unsigned failure_line;
    float pending_target;
    float origin[3], command[2], baseline_radius;
    float sum[3], minimum[3], maximum[3], endpoints[4][3];
    float base_minimum, base_maximum;
    int pole_x, pole_y, target_x, target_y;
    float gradient[2];
    float measured_matrix[4], radius_delta[2];
} vision_calibrate_t;
void vision_calibrate_start(vision_calibrate_t *s);
/* Returns a bounded shoulder/elbow proposal. No guessed image derivatives. */
bool vision_calibrate_step(vision_calibrate_t *s, const fpga_frame_t *f,
    bool fresh, bool healthy, bool permitted, unsigned dt_ms,
    const float measured[3], const float accepted[2], float proposal[2]);
#endif
