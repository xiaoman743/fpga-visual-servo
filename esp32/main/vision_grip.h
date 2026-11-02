#ifndef VISION_GRIP_H
#define VISION_GRIP_H
#include <stdbool.h>
typedef enum {VG_OFF,VG_SEEK,VG_PRESS,VG_READY,VG_FAILED} vision_grip_state_t;
typedef struct {
    vision_grip_state_t state;
    float target,floor,contact,last_measured;
    unsigned elapsed_ms,stable_ms,press_ms,bad_ms,read_ms;
    unsigned late_cycles; /* 连续超时周期；单次卡顿只冻结，不直接判失败 */
    bool have_measurement;
} vision_grip_t;
bool vision_grip_begin(vision_grip_t *s,float accepted,float measured,float floor);
/* Targets only decrease (close). Must be fed fresh healthy CH6 measurements.
 * Position resistance is evidence of obstruction, not proof of object identity. */
void vision_grip_step(vision_grip_t *s,bool healthy,bool fresh,unsigned dt_ms,
    float accepted,float measured);
#endif
