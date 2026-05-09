#ifndef __PID_H_
#define __PID_H_
#include "zf_common_headfile.h"
typedef struct {
    int16 target_val;   /* 目标速度 */
    int16 actual_val;   /* 实际速度 */
    int16 err;          /* 当前误差 e(k) */
    int16 err_last;     /* 上一次误差 e(k-1) */
    int16 err_prev;     /* 上上次误差 e(k-2) */
    
    /* 
     * 放大后的 PID 参数。
     * 例如真实 Kp 为 2.5，则存储为 250 (放大100倍)
     */
    int16 Kp;
    int16 Ki;
    int16 Kd;
    
    int32 output_val;   /* 最终输出的控制量 (如 PWM 占空比) */
} PID_t;

void PID_Init(PID_t *pid, int16 p, int16 i, int16 d);
int16 PID_Speed(PID_t *pid, int16 current_speed);

#endif