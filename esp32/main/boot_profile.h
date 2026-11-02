#ifndef BOOT_PROFILE_H
#define BOOT_PROFILE_H
#include "rt_level.h"
#include "rt_height.h"
bool boot_profile_restore(rt_level_t *level,rt_height_t *height,
    const float cmd[3],const float measured[2],const float lo[3],const float hi[3]);
#define BOOT_LIFT_DEGREES 45.0f
/* Recover only the 40-tick EEPROM margin, always toward its interior.
 * Returns -1 outside physical limits, 0 already inside, 1 bounded recovery. */
int boot_profile_margin_step(const float cmd[3],const float measured[3],
    const float lo[3],const float hi[3],float max_step,float next[3]);
#endif
