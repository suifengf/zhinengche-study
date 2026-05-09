#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "zf_common_headfile.h"
#define ENCODER_QUAD_1                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_QUAD_1_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_QUAD_1_CHB              (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

#define ENCODER_QUAD_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_2_CHA   		    (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_QUAD_2_CHB       	    (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

#define PWM_LEFT                 (PWMB_CH3_P76)
#define PWM_RIGHT                (PWMD_CH2_P51)
void Motor_Init(void);
#endif
