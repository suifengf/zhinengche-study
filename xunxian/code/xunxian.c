#include "xunxian.h"
#define O1  IO_P54
#define O2  IO_P94
#define O3  IO_P92
#define O4  IO_P93

uint8 key1_status = 1;
uint8 key1_last_status;
uint8 key1_flag;


void xunxian_init(void)
{
    gpio_init(O1, GPI, 1, GPI_PULL_UP);
    gpio_init(O2, GPI, 1, GPI_PULL_UP);
    gpio_init(O3, GPI, 1, GPI_PULL_UP);
    gpio_init(O4, GPI, 1, GPI_PULL_UP);
}

void xunxian_scan(uint8* color)
{
    color[0] = gpio_get_level(O1);
    color[1] = gpio_get_level(O2);
    color[2] = gpio_get_level(O3);
    color[3] = gpio_get_level(O4);
}

uint8 run(uint8 flag)
{
    key1_last_status = key1_status;
    key1_status = gpio_get_level(KEY1_PIN);
    if(key1_status && !key1_last_status)    key1_flag = 1;
    if(key1_flag)   
    {
        key1_flag = 0;//使用按键之后，应该清除标志位
        flag = !flag;
        
    }
    return flag;
}


void buzzer_on(void)
{
    gpio_set_level(BEEP_PIN, GPIO_HIGH);
}

void buzzer_off(void)
{
    gpio_set_level(BEEP_PIN, GPIO_LOW);
}