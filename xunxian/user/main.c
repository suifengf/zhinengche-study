#include "zf_common_headfile.h"
#include "xunxian.h"
#include "Pid.h"
#include "Motor.h"
#include "Key.h"
#define PIT_PID                  (TIM0_PIT )                	// 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_KEY                  (TIM1_PIT)                     
#define PIT_PRIORITY_PID            (TIMER0_IRQn)              	// 对应周期中断的中断编号
#define PIT_PRIORITY_KEY            (TIMER1_IRQn)              	// 对应周期中断的中断编号

#define SERVO_PWM1              (PWME_CH1P_PA0)
#define SERVO_FREQ              (50)
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#define PID_P                      (50)
#define PID_I                      (15)
#define PID_D                      (0)

#define PIT                         (TIM6_PIT )

#define Speed                       (1250) // 目标速度，单位为编码器计数每周期

uint8 Key_State = 0;
uint8 Key_Data[5] = {0};
uint8 pit_state = 0;
uint8 color[4];

uint8  pit_ms_flag          = 0;
uint16  pit_ms_count         = 0;
uint16  dl1b_refresh_freq    = 0;

int16 far encoder_data_right = 0;
int16 far encoder_data_left = 0;

int16 pwm_right = 0;
int16 pwm_left = 0;

PID_t pid_r,pid_l;

uint8 run_flag;

void pit_handler_pid (void);
void pit_handler_key (void);
void pit_handler_tof (void);
void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    xunxian_init();
    pit_ms_init(PIT_PID, 10, pit_handler_pid);                                      // 初始化 PIT 为周期中断 10ms 周期
    pit_ms_init(PIT_KEY, 1, pit_handler_key);                                      // 初始化 PIT 为周期中断 10ms 周期
    Motor_Init();
    dl1b_init();
    PID_Init(&pid_r, PID_P, PID_I, PID_D); // 初始化右轮 PID 参数
    PID_Init(&pid_l, PID_P, PID_I, PID_D); // 初始化左轮 PID 参数
    pwm_init(SERVO_PWM1,SERVO_FREQ,SERVO_DUTY(90));
    tft180_init();
    pid_l.target_val = Speed;
    pid_r.target_val = Speed;
    gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
    run_flag = 0;


    pit_ms_init(PIT, 5, pit_handler_tof);

    // 此处编写用户代码
    while(1) 
    {
        // xunxian_scan(color);
        // printf("%d,%d,%d,%d\r\n",color[0],color[1],color[2],color[3]);
        Key_Proc (&Key_State, Key_Data);
        tft180_show_int16(10, 10, pwm_right);
        tft180_show_int16(10, 30, pwm_left);
        tft180_show_int16(10, 50, encoder_data_right);
        tft180_show_int16(10, 70, encoder_data_left);
        tft180_show_int16(10, 90, run_flag);
        tft180_show_int16(10, 110, Key_Data[0]);

        if(Key_Data[0] == 1)
        {
            run_flag = 1;
        }
        else
        {
            run_flag = 0;
        }

        if(run_flag == 0)
        {
            Motor_Stop();
        }

        if(pit_state && run_flag)
        {
            pit_state = 0;                                                          // 周期中断触发 标志位清零
            get_encoder_count_right(&encoder_data_right);
            get_encoder_count_left(&encoder_data_left);
            pwm_right = PID_Speed(&pid_r, encoder_data_right); // 计算右轮 PWM 输出
            pwm_left = PID_Speed(&pid_l, encoder_data_left); // 计算左轮 PWM 输出
            // ---------- 动态控制右轮 ----------
            Motor_Right_Run(pwm_right);
            // ---------- 动态控制左轮 ----------
            Motor_Left_Run(pwm_left);

            
        }

        if(pit_ms_flag)
        {
            printf("\r\n DL1B distance data: %5d \r\n", dl1b_distance_mm);
            printf("\r\n DL1B refresh freq: %5d \r\n", dl1b_refresh_freq);
            dl1b_refresh_freq = 0;
            pit_ms_flag = 0;
        }
    }
}

// **************************** 代码区域 ****************************


void pit_handler_pid (void)
{
    pit_state = 1;                                                              // 周期中断触发 标志位置位
}

void pit_handler_tof (void)
{
    // 如果开启了 INT 的使能 那么会自动触发外部中断进行数据采集 就不需要自行采集了
#if (!DL1B_INT_ENABLE)
    if(0 == pit_ms_count % 10)                                                  // 每 10ms 获取一次测距信息 周期 10 ms 频率 100Hz
    {
        dl1b_get_distance();                                                    // 测距调用频率不应高于 100Hz 周期不应低于 10ms
    }
#endif
    // 通过判断 dl1a_finsh_flag 来确定是否更新了数据 记得获取了数据后就清空标志位
    if(dl1b_finsh_flag)
    {
        dl1b_refresh_freq ++;
        dl1b_finsh_flag = 0;
    }
    pit_ms_count = (pit_ms_count == 995) ? (0) : (pit_ms_count + 5);            // 1000ms 周期计数
    if(0 == pit_ms_count % 1000)
    {
        pit_ms_flag = 1;
    }
}
