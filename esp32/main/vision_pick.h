#ifndef VISION_PICK_H
#define VISION_PICK_H
#include <stdbool.h>
typedef enum {VP_IDLE,VP_SETTLE,VP_LIFT,VP_DONE,VP_ABORT} vision_pick_state_t;
typedef struct {
    vision_pick_state_t state;
    bool armed,was_closed;
    float degrees,start,target,last_accepted,measured_start,height_start;
    unsigned elapsed_ms,blocked_ms,verified_ms,unhealthy_ms;
    unsigned late_cycles; /* 连续超时周期；单次卡顿只冻结，不直接中止抬升 */
} vision_pick_t;
bool vision_pick_arm(vision_pick_t *p,float degrees);
bool vision_pick_resume(vision_pick_t *p,float degrees);
void vision_pick_cancel(vision_pick_t *p);
// accepted is the command after the common constraints from the last cycle.
bool vision_pick_step(vision_pick_t *p,bool closed,bool start_ok,bool healthy,
                      float accepted,float measured,float q3,float off2,float off3,float sign,
                      unsigned dt_ms,float *proposal);
#endif
