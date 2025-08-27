#include "Car_main.h"
#include "include.h"
#include "stdlib.h"

#define WAY_Flag 0 // ÈüµÀÑ¡Ôñ£¬1:ºÚµ×°×Ïß 0:°×µ×ºÚÏß

Car Target_V; // Ä¿±ê×ªËÙ
Car ENC_V; // Êµ¼Ê×ªËÙ
Car Moto_PWM; // ¿ØÖÆµç»ú¼ÆËã³öµÄPWM

uint8_t Motor_Flag = 0; // µç»úÆôÍ£±êÖ¾Î» 0£ºÍ£Ö¹ÔËÐÐ 1£º¿ªÊ¼ÔË¶¯
float E_V = 0.0;
int32_t car_V;
sensor Car_sensor;

int state = 0;

//??1
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
} PID_Controller;

PID_Controller pid = {200.0, 0.0, 0.0, 0.0, 0.0}; // ??PID??
volatile uint8_t sensor_updated = 0; // ???????

void Car_main(void)
{
    LED_Init(); //³õÊ¼»¯LED
    KEY_Init(); //³õÊ¼»¯°´¼ü
    OLED_Init(); //OLED³õÊ¼»¯
    OLED_CLS(); //ÇåÆÁ
    uart_init(USART_2, 115200); //³õÊ¼»¯´®¿Ú
    uart_init(USART_3, 115200); //³õÊ¼»¯´®¿Ú
    Encoder_Init_TIM2(); //±àÂëÆ÷³õÊ¼»¯
    Encoder_Init_TIM3(); //±àÂëÆ÷³õÊ¼»¯
    Encoder_Init_TIM4(); //±àÂëÆ÷³õÊ¼»¯
    MotorInit(); //µç»ú³õÊ¼»¯
    sensor_init(); //¹âµç´«¸ÐÆ÷³õÊ¼»¯
    
    while(1)
    {
        if (Read_key(KEY1) == 1)
        {
            if (Motor_Flag == 1)
                Motor_Flag = 0;
            else if (Motor_Flag == 0)
                Motor_Flag = 1;
        }
        OLED_Task(); //ÆÁÄ»ÏÔÊ¾
    }
}

/*********************************************************************
* º¯ÊýÃû £ºvoid car_tim(void)
* ²Î Êý £ºÎÞ
* ·µ»ØÖµ £ºÎÞ
* ¹¦ ÄÜ £ºÈýÂÖÐ¡³µ Ñ­Ïß³ÌÐò
* Ëµ Ã÷ £ºÌá¹©Á½ÖÖÈüµÀ£¬°×ÏßºÚµ×¡¢ºÚÏß°×µ×£¬WAY_Flag = 1:ºÚµ×°×Ïß 0:°×µ×ºÚÏß
* µ×É«ÓëÏßÉ«²î´ó¼´¿É£¬¾ßÌå¿ÉÒÔµ÷½Ú¹âµçÁéÃô¶È
********************************************************************/
void car_tim(void)
{
    // ??????????????
    static float output;
    float error;
    float derivative;

#if WAY_Flag
    Car_sensor.a = 1 - Read_sensor(sensor2);
    Car_sensor.b = 1 - Read_sensor(sensor3);
    Car_sensor.c = 1 - Read_sensor(sensor4);
    Car_sensor.d = 1 - Read_sensor(sensor1);
#else
    Car_sensor.a = -(1 - Read_sensor(sensor2));
    Car_sensor.b = -(1 - Read_sensor(sensor3));
    Car_sensor.c = -(1 - Read_sensor(sensor4));
    Car_sensor.d = -(1 - Read_sensor(sensor1));
#endif
    
    E_V = (Car_sensor.a * 2 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 2);
    
    // PID??
    error = E_V;
    pid.integral += error;
    
    // ????
    if (pid.integral > 1000) pid.integral = 1000;
    if (pid.integral < -1000) pid.integral = -1000;
    
    derivative = error - pid.previous_error;
    output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.previous_error = error;
    //??2

#if WAY_Flag
    if (Car_sensor.a == 1 && Car_sensor.b == 1 && Car_sensor.c == 1 && Car_sensor.d == 1)
    {
        car_V = -800;
    }
    else
    {
        if (abs(E_V) > 2) // ??
            car_V = 900;
        else // ??
            car_V = 1200;
    }
#else
    if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
    {
        car_V = -800;
    }
    else
    {
		if(abs(E_V) != 0)
		{
			car_V = 0;
		}
    }
#endif
    
    if (Motor_Flag)
    {
		if(abs(E_V) == 0)
		{
			Moto_PWM.L = car_V + output;
        	Moto_PWM.R = -car_V + output;
        	Moto_PWM.B = output * 1.15;	
		}
		else
		{
			Moto_PWM.L = car_V + output;
        	Moto_PWM.R = -car_V + output;
        	Moto_PWM.B = output;
		}
    }
    else // ??????
    {
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
    }
    // ????
    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L); // ??????
}


void OLED_Task(void)
{
    char txt[64];
    
    // ????
    sprintf(txt, "%d %d %d %d E:%.1f", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
    OLED_P6x8Str(0, 2, txt); // ???
    sprintf(txt, "PWM: L: %5d", Moto_PWM.L);
    OLED_P6x8Str(0, 3, txt); // ???
    sprintf(txt, "PWM: R: %5d", Moto_PWM.R);
    OLED_P6x8Str(0, 4, txt); // ???
    sprintf(txt, "PWM: B: %5d", Moto_PWM.B);
    OLED_P6x8Str(0, 5, txt); // ???
    
    switch (state)
    {
        case 1:
            sprintf(txt, "R");
            break;
        case 2:
            sprintf(txt, "L");
            break;
        case 3:
            sprintf(txt, "F");
            break;
        case 0:
            sprintf(txt, "B");
    }
    OLED_P6x8Str(0, 6, txt); // ???

    delay_ms(100);
}