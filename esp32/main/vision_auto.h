#ifndef VISION_AUTO_H
#define VISION_AUTO_H
#include <stdbool.h>
#include <stdint.h>
#include "vision_frame.h"
typedef enum {VA_IDLE,VA_WAIT,VA_ALIGN,VA_REACH,VA_SETTLE,VA_CLOSE,VA_LIFT,VA_DONE,VA_ABORT} vision_auto_state_t;
typedef struct {
    vision_auto_state_t state;
    float lift_degrees, radial_offset;
    unsigned elapsed_ms, stable_ms, unhealthy_ms, vision_stale_ms;
    unsigned stable_frames;
    unsigned shifted_frames;
    unsigned late_cycles; /* 连续超时周期数；达到上限才算主循环真的卡死 */
    unsigned stall_ms;    /* 目标顶在领先窗口边界、实测不动 的累计毫秒（领窗死锁检测） */
    bool have_config, close_issued;
    bool confirmed_endpoint, key3_was_off;
    int16_t pole_x,pole_y;
    uint16_t target_x,target_y;
    const char *reason;
} vision_auto_t;
typedef struct {
    bool configured, selected, constraints_ready, healthy, manual_override;
    bool frame_fresh, new_frame, map_ready, jaws_open, gripper_settled, calibrating;
    bool lift_done, lift_aborted;
    float joint_speed, measured_motion, position_error;
    float expected_offset;
    unsigned dt_ms;
    const fpga_frame_t *frame;
} vision_auto_input_t;
typedef struct { bool pan,reach,close,arm_lift,freeze,calibrate; } vision_auto_output_t;
bool vision_auto_arm(vision_auto_t *s,float degrees);
bool vision_auto_finish(vision_auto_t *s,float degrees);
bool vision_auto_resume_lift(vision_auto_t *s,float degrees,const fpga_frame_t *frame);
void vision_auto_cancel(vision_auto_t *s,const char *reason);
bool vision_auto_running(const vision_auto_t *s);
vision_auto_output_t vision_auto_step(vision_auto_t *s,const vision_auto_input_t *in);
const char *vision_auto_name(vision_auto_state_t state);
#endif
