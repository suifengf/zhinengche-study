#include "Motor.h"

void Motor_Init(void)
{
    //B组，左边
    gpio_init(IO_P74, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P75, GPO, 0, GPO_PUSH_PULL);
    //A组，右边
    gpio_init(IO_P50, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P52, GPO, 0, GPO_PUSH_PULL);

    pwm_init(PWM_LEFT, 10000, 0);      //控制轮子转速               
    pwm_init(PWM_RIGHT, 10000, 0);     //控制轮子转速

    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
}



