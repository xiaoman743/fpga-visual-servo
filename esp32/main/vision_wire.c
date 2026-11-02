#include "vision_wire.h"
static int hex(uint8_t c) {
    if(c>='0'&&c<='9')return c-'0';
    if(c>='A'&&c<='F')return c-'A'+10;
    return -1;
}
static uint16_t u16(const uint8_t *p){return (uint16_t)((uint16_t)p[0]<<8|p[1]);}
bool vision_wire_decode(const uint8_t *line,size_t len,vision_wire_t *out) {
    if(!line||!out||len!=VISION_WIRE_LEN||line[0]!='@'||line[57]!='\r'||line[58]!='\n')return false;
    uint8_t d[28];
    for(unsigned i=0;i<28;i++) {
        int a=hex(line[1+2*i]),b=hex(line[2+2*i]);
        if(a<0||b<0)return false;
        d[i]=(uint8_t)(a*16+b);
    }
    uint16_t crc=0xffff;
    for(unsigned i=0;i<26;i++) {
        crc^=(uint16_t)d[i]<<8;
        for(unsigned j=0;j<8;j++)crc=(uint16_t)((crc<<1)^((crc&0x8000)?0x1021:0));
    }
    if(d[0]!=2||(!(d[1]&0x80)&&(d[1]&0x60))||crc!=u16(d+26))return false;
    vision_wire_t v={.flags=d[1],.sequence=u16(d+2),.red_x=u16(d+4),.red_y=u16(d+6),
        .blue_x=u16(d+8),.blue_y=u16(d+10),.goal_x=u16(d+12),.goal_y=u16(d+14),
        .pole_x=(int16_t)u16(d+16),.pole_y=(int16_t)u16(d+18),.gap=(int16_t)u16(d+20),
        .pan=(int16_t)u16(d+22),.reach=(int16_t)u16(d+24)};
    if(v.red_x>=800||v.blue_x>=800||v.goal_x>=800||v.red_y>=480||v.blue_y>=480||v.goal_y>=480||
       v.pole_x< -2048||v.pole_x>2047||v.pole_y< -2048||v.pole_y>2047||
       v.gap< -200||v.gap>200||v.pan< -8||v.pan>8||v.reach< -8||v.reach>8)return false;
    if((v.flags&0x17)!=0x17 && (v.pan!=0||v.reach!=0))return false;
    *out=v;return true;
}
