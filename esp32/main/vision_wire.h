#ifndef VISION_WIRE_H
#define VISION_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define VISION_WIRE_LEN 59
typedef struct {
    uint8_t flags;
    uint16_t sequence, red_x,red_y,blue_x,blue_y,goal_x,goal_y;
    int16_t pole_x,pole_y,gap,pan,reach;
} vision_wire_t;
bool vision_wire_decode(const uint8_t *line,size_t len,vision_wire_t *out);
#endif
