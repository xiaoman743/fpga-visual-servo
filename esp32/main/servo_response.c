#include "servo_response.h"
#include <string.h>
static void discard(servo_response_t *s,unsigned n) {
    s->used-=n;memmove(s->data,s->data+n,s->used);
}
bool servo_response_feed(servo_response_t *s,uint8_t byte,uint8_t want_id,
                         unsigned expected,uint8_t *payload,uint8_t *error) {
    if(!s||expected>8||(expected&&!payload))return false;
    if(s->used>=sizeof(s->data))discard(s,1);
    s->data[s->used++]=byte;
    while(s->used>=4) {
        uint8_t *b=s->data;
        if(b[0]!=255||b[1]!=255){discard(s,1);continue;}
        unsigned len=b[3],total=len+4;
        if(len<2||len>10){s->rejected++;discard(s,1);continue;}
        if(s->used<total)return false;
        unsigned sum=0;
        for(unsigned i=2;i<total;i++)sum+=b[i];
        if((sum&255)!=255){s->rejected++;discard(s,1);continue;}
        if(len!=expected+2){s->rejected++;discard(s,total);continue;}
        if(b[2]!=want_id){s->wrong_id++;discard(s,total);continue;}
        if(b[4]!=0){s->device_errors++;s->last_error=b[4];s->rejected++;discard(s,total);continue;}
        if(expected)memcpy(payload,b+5,expected);
        if(error)*error=b[4];
        discard(s,total);return true;
    }
    return false;
}
