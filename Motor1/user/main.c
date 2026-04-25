#include "zf_common_headfile.h"
#include "userheadfile.h"

uint8 wifi_spi_test_buffer[] = "Connect";
uint16 data_length = 0;

uint8 wifi_spi_get_data_buffer[256] = {0};
uint32 wifi_cmd_timeout = 0; 

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();
    
	pwm_init(SERVO_PWM1,SERVO_FREQ,SERVO_DUTY(90));
    pwm_init(PWM_CH1, 10000, 0);      //控制轮子转速               
    pwm_init(PWM_CH2, 10000, 0);               
    pwm_init(PWM_CH3, 10000, 0);
    pwm_init(PWM_CH4, 10000, 0);     //控制轮子转速              
    pwm_init(PWM_CH5, 10000, 0);               
    pwm_init(PWM_CH6, 10000, 0);
    
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s \r\n",wifi_spi_version);                     // 模块固件版本
    printf("\r\n module mac    :%s \r\n",wifi_spi_mac_addr);                    // 模块 MAC 信息
    printf("\r\n module ip     :%s \r\n",wifi_spi_ip_addr_port);                // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接、创建 TCP 服务器等操作
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again. \r\n");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }

    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）

    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    if(!data_length)
    {
        printf("\r\n send success. \r\n");
    }
    else
    {
        printf("\r\n %d bytes data send failed. \r\n", data_length);
    }
    


    // 此处编写用户代码
    while(1)
    {
		data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
        if(data_length)                                                     // 如果接收到数据 则进行数据类型判断
        {
			wifi_cmd_timeout = 0;
            printf("\r\n Get data: <%s>. \r\n", wifi_spi_get_data_buffer);
            switch (wifi_spi_get_data_buffer[0])
            {
                case 'D':
                {
                    pwm_set_duty(PWM_CH1,1500);
                    pwm_set_duty(PWM_CH2,10000);
                    pwm_set_duty(PWM_CH3,0);
                    pwm_set_duty(PWM_CH4,1500);
                    pwm_set_duty(PWM_CH5,10000);
                    pwm_set_duty(PWM_CH6,0);
                    
                    break;
                }
                case 'U':
                {
                    pwm_set_duty(PWM_CH1,1500);
                    pwm_set_duty(PWM_CH2,0);
                    pwm_set_duty(PWM_CH3,10000);
                    pwm_set_duty(PWM_CH4,1500);
                    pwm_set_duty(PWM_CH5,0);
                    pwm_set_duty(PWM_CH6,10000);
                    
                    break;
                }
                case 'R':
                {
                    pwm_set_duty(SERVO_PWM1,SERVO_DUTY(80));
                    pwm_set_duty(PWM_CH1,1500);
                    pwm_set_duty(PWM_CH2,0);
                    pwm_set_duty(PWM_CH3,10000);
                    pwm_set_duty(PWM_CH4,1500);
                    pwm_set_duty(PWM_CH5,0);
                    pwm_set_duty(PWM_CH6,10000);
                    break;
                }
                case 'L':
                {
                    pwm_set_duty(SERVO_PWM1,SERVO_DUTY(100));
                    pwm_set_duty(PWM_CH1,1500);
                    pwm_set_duty(PWM_CH2,0);
                    pwm_set_duty(PWM_CH3,10000);
                    pwm_set_duty(PWM_CH4,1500);
                    pwm_set_duty(PWM_CH5,0);
                    pwm_set_duty(PWM_CH6,10000);
                    break;
                }
                

            }
        }
        else
        {
            wifi_cmd_timeout++;
            if(wifi_cmd_timeout > 4000)     // 如果超过一定时间没有接
            {   
                pwm_set_duty(SERVO_PWM1,SERVO_DUTY(90));
                pwm_set_duty(PWM_CH1,0);
                pwm_set_duty(PWM_CH2,0);
                pwm_set_duty(PWM_CH3,0);
                pwm_set_duty(PWM_CH4,0);
                pwm_set_duty(PWM_CH5,0);
                pwm_set_duty(PWM_CH6,0);
				
            }
            
        }
        memset(wifi_spi_get_data_buffer, 0, data_length);

        



    }
}

// **************************** 代码区域 ****************************



