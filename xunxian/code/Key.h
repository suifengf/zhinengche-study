#ifndef __KEY_H_
#define __KEY_H_
#include "zf_common_headfile.h"
typedef struct
{
	uint8 up;
	uint8 down;
	uint8 val;
	uint8 old;
	uint8 state;
	uint32 delay;
}Key_t;

#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32

#define KEY_TIM_LONG 1000
#define KEY_TIM_DOUBLE 500
#define KEY_TIM_PRESSING 300


#define KEY_STATE_DOWN 1
#define KEY_STATE_PRESSED 2
#define KEY_STATE_LONG 3
#define KEY_STATE_PERESSING 4
#define KEY_STATE_DOUBLE 5

void Key_Proc (uint8* Key,uint8* Data);

#endif
