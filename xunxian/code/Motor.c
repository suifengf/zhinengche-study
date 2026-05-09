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

void Motor_Stop(void)
{
    gpio_set_level(IO_P74, GPIO_LOW); // 左轮停止
    gpio_set_level(IO_P75, GPIO_LOW);

    gpio_set_level(IO_P50, GPIO_LOW); // 右轮停止 
    gpio_set_level(IO_P52, GPIO_LOW);

    pwm_set_duty(PWM_RIGHT, 0);
    pwm_set_duty(PWM_LEFT, 0);
}

void Motor_Right_Run(int32 pwm_r)
{
    if(pwm_r >= 0)
    {
        gpio_set_level(IO_P50, GPIO_HIGH); 
        gpio_set_level(IO_P52, GPIO_LOW);
    }
    else
    {
        gpio_set_level(IO_P50, GPIO_LOW);  
        gpio_set_level(IO_P52, GPIO_HIGH);
        pwm_r = -pwm_r; // 设置好反转后，再提取绝对值给 PWM 寄存器
    }
    pwm_set_duty(PWM_RIGHT, pwm_r);
}

void Motor_Left_Run(int32 pwm_l)
{
    if(pwm_l >= 0)
    {
        gpio_set_level(IO_P74, GPIO_HIGH); 
        gpio_set_level(IO_P75, GPIO_LOW);
    }
    else
    {
        gpio_set_level(IO_P74, GPIO_LOW);  
        gpio_set_level(IO_P75, GPIO_HIGH);
        pwm_l = -pwm_l; // 设置好反转后，再提取绝对值
    }
    pwm_set_duty(PWM_LEFT, pwm_l);
}

void get_encoder_count_left(uint16* count_l)
{
    *count_l = encoder_get_count(ENCODER_QUAD_2);
    encoder_clear_count(ENCODER_QUAD_2);
}

void get_encoder_count_right(uint16* count_r)
{
    *count_r = -encoder_get_count(ENCODER_QUAD_1);
    encoder_clear_count(ENCODER_QUAD_1);
}