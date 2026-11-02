#ifndef VISION_REACH_H
#define VISION_REACH_H
#include <stdbool.h>
#define VISION_REACH_SAMPLES 32
typedef struct {float q1,q2,q3,dr2,dr3;bool used;} vision_reach_sample_t;
typedef struct {
    vision_reach_sample_t samples[VISION_REACH_SAMPLES];
    unsigned count;
} vision_reach_map_t;
typedef struct {bool ready;float q1,q2,q3,radius,height;} vision_reach_track_t;
void vision_reach_clear(vision_reach_map_t *m);
bool vision_reach_add(vision_reach_map_t *m,float q1,float q2,float q3,float dr2,float dr3);
bool vision_reach_gradient(const vision_reach_map_t *m,float q1,float q2,float q3,float *gr2,float *gr3);
// Learn only from observed motion wholly inside an existing sample region.
bool vision_reach_observe(vision_reach_map_t *m,vision_reach_track_t *t,
    float q1,float q2,float q3,float radius,float off2,float off3,float sign);
// Image gradients are local and valid only within 100 ticks in joint space.
// Height gradients are recomputed at the measured current pose every step.
bool vision_reach_solve(const vision_reach_map_t *m,float q1,float q2,float q3,
                        float off2,float off3,float sign,float dr_px,float *dq2,float *dq3);
/* 带限位的求解：某个轴已到边界时，把剩下的径向需求转给另一个轴，
 * 避免“CH2 顶到上限就整段卡住、CH3 还有余量却不参与”。 */
bool vision_reach_solve_limited(const vision_reach_map_t *m,float q1,float q2,float q3,
    float off2,float off3,float sign,float dr_px,
    float lo2,float hi2,float lo3,float hi3,float *dq2,float *dq3);
/* 高度模式由主循环每周期喂入：0 等高切线 / 1 允许边伸边抬 / 2 已超上限只许下降。
 * 求解器本身不知道绝对高度，必须由执行侧把天花板提前告诉它，否则会在上限处死锁。 */
void vision_reach_set_height_mode(int mode);
int vision_reach_height_mode(void);
#endif
