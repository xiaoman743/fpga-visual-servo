#ifndef SERVO_RESPONSE_H
#define SERVO_RESPONSE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint8_t data[32],last_error;
    unsigned used,wrong_id,rejected,device_errors;
} servo_response_t;
// Current transactions read at most eight bytes. Reject echoes, error replies,
// and oversized payloads before copying into the caller's eight-byte buffer.
bool servo_response_feed(servo_response_t *s,uint8_t byte,uint8_t want_id,
                         unsigned expected,uint8_t *payload,uint8_t *error);
#endif
