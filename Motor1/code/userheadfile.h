#ifndef __USERHEADFILE_H_
#define __USERHEADFILE_H_

#define SERVO_PWM1              (PWME_CH1P_PA0)
#define SERVO_FREQ              (50)
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#if ((SERVO_FREQ < 50) || (SERVO_FREQ > 300))
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

#define PWM_CH1                 (PWMB_CH1_P74)
#define PWM_CH2                 (PWMB_CH2_P75)
#define PWM_CH3                 (PWMB_CH3_P76)
#define PWM_CH4                 (PWMD_CH1_P50)
#define PWM_CH5                 (PWMD_CH2_P51)
#define PWM_CH6                 (PWMD_CH3_P52)


#define WIFI_SSID_TEST          "@Ruijie-s4D50"
#define WIFI_PASSWORD_TEST      "gcxy2105"




#endif