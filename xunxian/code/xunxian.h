#ifndef __XUNXIAN_H_
#define __XUNXIAN_H_
#include "zf_common_headfile.h"
#define KEY1_PIN        IO_PB2

void xunxian_init(void);
void xunxian_scan(uint8* color);
uint8 run(uint8 flag);
#endif