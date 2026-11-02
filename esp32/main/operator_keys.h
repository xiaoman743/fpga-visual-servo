#ifndef OPERATOR_KEYS_H
#define OPERATOR_KEYS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {bool seen,enabled;uint8_t parity;int64_t last_us;} operator_keys_t;
/* First frame and reconnect establish a baseline, never synthesize a press. */
unsigned operator_keys_step(operator_keys_t *s,bool valid,uint8_t parity,bool enabled,int64_t now_us);
enum {OP_KEY_OPEN=1,OP_KEY_TORQUE=2,OP_KEY_START=4};
typedef enum {OP_NONE,OP_OPEN,OP_RELEASE,OP_HOLD,OP_START_RESTORE,OP_START_READY} operator_action_t;
operator_action_t operator_keys_action(unsigned keys,bool torque_on,bool service_hold);
#endif
