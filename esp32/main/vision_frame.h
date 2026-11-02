#ifndef VISION_FRAME_H
#define VISION_FRAME_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    bool v2, visual_enabled, goal_valid, aligned;
    bool key_events_valid;
    uint8_t key_events;
    uint16_t wire_sequence, goal_x, goal_y;
    int16_t pole_x, pole_y, gap;
    uint16_t red_x; /* 0..799 */
    uint16_t red_y; /* 0..479 */
    uint16_t blue_x;
    uint16_t blue_y;
    bool found_red;
    bool found_blue;
    int16_t pan_inc;   /* P，tick/帧，已按符号还原 */
    int16_t reach_inc; /* E，tick/帧 */
    int64_t rx_us;     /* 本机 esp_timer 时间戳 */
    uint32_t seq;      /* 本模块累计的有效帧序号（从 1 起） */
} fpga_frame_t;
#endif
