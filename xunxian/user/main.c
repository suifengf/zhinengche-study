#include "zf_common_headfile.h"
#include "xunxian.h"
#include "Pid.h"
#define PIT_CH                  (TIM0_PIT )                	// 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (TIMER0_IRQn)              	// 对应周期中断的中断编号
#define ENCODER_QUAD_1                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_QUAD_1_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_QUAD_1_CHB              (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

#define ENCODER_QUAD_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_2_CHA   		    (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_QUAD_2_CHB       	    (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

#define PWM_LEFT                 (PWMB_CH3_P76)
#define PWM_RIGHT                (PWMD_CH2_P51)


#define SERVO_PWM1              (PWME_CH1P_PA0)
#define SERVO_FREQ              (50)
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#define PID_P                      (50)
#define PID_I                      (15)
#define PID_D                      (0)

#define Speed                       (1250) // 目标速度，单位为编码器计数每周期




uint8 pit_state = 0;
uint8 color[4];

int16 encoder_data_dir_1 = 0;
int16 encoder_data_dir_2 = 0;

int16 pwm_right = 0;
int16 pwm_left = 0;

PID_t pid_r,pid_l;

uint8 run_flag;


void pit_handler (void);

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    xunxian_init();
    pit_ms_init(PIT_CH, 10, pit_handler);                                      // 初始化 PIT 为周期中断 10ms 周期

    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式

    PID_Init(&pid_r, PID_P, PID_I, PID_D); // 初始化右轮 PID 参数
    PID_Init(&pid_l, PID_P, PID_I, PID_D); // 初始化左轮 PID 参数

    pwm_init(SERVO_PWM1,SERVO_FREQ,SERVO_DUTY(90));
    pwm_init(PWM_LEFT, 10000, 0);      //控制轮子转速               
    pwm_init(PWM_RIGHT, 10000, 0);     //控制轮子转速
    //B组，左边
    gpio_init(IO_P74, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P75, GPO, 0, GPO_PUSH_PULL);
    //A组，右边
    gpio_init(IO_P50, GPO, 0, GPO_PUSH_PULL);
    gpio_init(IO_P52, GPO, 0, GPO_PUSH_PULL);

    tft180_init();

    pid_l.target_val = Speed;
    pid_r.target_val = Speed;
    
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    run_flag = 0;

    // 此处编写用户代码
    while(1) 
    {
        // xunxian_scan(color);
        // printf("%d,%d,%d,%d\r\n",color[0],color[1],color[2],color[3]);

        gpio_set_level(IO_P74, GPIO_HIGH); // 左轮正转
        gpio_set_level(IO_P75, GPIO_LOW);

        gpio_set_level(IO_P50, GPIO_HIGH); // 右轮正转 
        gpio_set_level(IO_P52, GPIO_LOW);

        pwm_set_duty(PWM_RIGHT,1500);
        pwm_set_duty(PWM_LEFT,1500);

        tft180_show_int16(10, 50, encoder_data_dir_1);
        tft180_show_int16(10, 70, encoder_data_dir_2);

        
        if(pit_state)
        {
            pit_state = 0;                                                          // 周期中断触发 标志位清零
            encoder_data_dir_1 = -encoder_get_count(ENCODER_QUAD_1);                  // 获取编码器计数
            encoder_data_dir_2 = encoder_get_count(ENCODER_QUAD_2);                  // 获取编码器计数
            encoder_clear_count(ENCODER_QUAD_1);                                		// 清空编码器计数
            encoder_clear_count(ENCODER_QUAD_2);                                		// 清空编码器计数

            pwm_right = PID_Speed(&pid_r, encoder_data_dir_1); // 计算右轮 PWM 输出
            pwm_left = PID_Speed(&pid_l, encoder_data_dir_2); // 计算左轮 PWM 输出

            if (pwm_right >= 0) 
            {
                // 如果 PID 需要正向发力
                gpio_set_level(IO_P50, GPIO_HIGH); 
                gpio_set_level(IO_P52, GPIO_LOW);
            } 
            else 
            {
                // 如果 PID 需要反向发力（刹车或倒车）
                gpio_set_level(IO_P50, GPIO_LOW);  
                gpio_set_level(IO_P52, GPIO_HIGH);
                pwm_right = -pwm_right; // 设置好反转后，再提取绝对值给 PWM 寄存器
            }

            // ---------- 动态控制左轮 ----------
            if (pwm_left >= 0) 
            {
                // 左轮正向发力
                gpio_set_level(IO_P74, GPIO_HIGH); 
                gpio_set_level(IO_P75, GPIO_LOW);
            } 
            else 
            {
                // 左轮反向发力
                gpio_set_level(IO_P74, GPIO_LOW);  
                gpio_set_level(IO_P75, GPIO_HIGH);
                pwm_left = -pwm_left; // 设置好反转后，再提取绝对值
            }

            
            pwm_set_duty(PWM_RIGHT, pwm_right);
            pwm_set_duty(PWM_LEFT, pwm_left);

            tft180_show_int16(10, 10, pwm_right);
            tft180_show_int16(10, 30, pwm_left);

        }
    }
}

// **************************** 代码区域 ****************************


void pit_handler (void)
{
    pit_state = 1;                                                              // 周期中断触发 标志位置位
}
