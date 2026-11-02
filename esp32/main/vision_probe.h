#ifndef VISION_PROBE_H
#define VISION_PROBE_H
#include <stdbool.h>
typedef struct {bool active;unsigned axis;float target,elapsed_ms;} vision_probe_t;
bool vision_probe_arm(vision_probe_t *s,unsigned axis,float current,float delta);
bool vision_probe_step(vision_probe_t *s,float dt_ms,bool permitted,bool healthy,
                       float accepted,float *proposal);
#endif
