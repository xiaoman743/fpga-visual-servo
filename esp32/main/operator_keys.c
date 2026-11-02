#include "operator_keys.h"
operator_action_t operator_keys_action(unsigned keys,bool torque_on,bool service_hold) {
    if(keys&OP_KEY_TORQUE)return torque_on?OP_RELEASE:OP_HOLD;
    if(keys&OP_KEY_OPEN)return OP_OPEN;
    if((keys&OP_KEY_START)&&service_hold)return torque_on?OP_START_READY:OP_START_RESTORE;
    return OP_NONE;
}
unsigned operator_keys_step(operator_keys_t *s,bool valid,uint8_t parity,bool enabled,int64_t now) {
    if(!valid){s->seen=false;return 0;}
    unsigned result=0;
    if(s->seen&&now>s->last_us&&now-s->last_us<=250000) {
        result=(s->parity^parity)&3;
        if(enabled&&!s->enabled)result|=OP_KEY_START;
    }
    s->seen=true;s->parity=parity;s->enabled=enabled;s->last_us=now;
    return result;
}
