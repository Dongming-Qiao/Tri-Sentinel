#include "Car_main.h"
#include "include.h"
#include <stdio.h>

extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;
int speed = 2000;

enum
{
    RUN = 0,
    MEET_FACE,
    SEARCH_EXIT_LEFT,
    SEARCH_EXIT_RIGHT,
    BACK_TO_FORWARD,
    DASH,
    AVOIDING,
    ROTATE
} Car_State;

const int LEFT_ROTATE_TIMES_MAX = 6;
int left_rotate_times = LEFT_ROTATE_TIMES_MAX;

const int RIGHT_ROTATE_TIMES_MAX = 12;
int right_rotate_times = RIGHT_ROTATE_TIMES_MAX;

const int BACK_TO_FORWARD_TIMES_MAX = 6;
int back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;

const int DASH_TIMES_MAX = 5;
int dash_times = DASH_TIMES_MAX;

const int ROTATE_TIME_MAX = 5;
int rotate_time = ROTATE_TIME_MAX;

const int DETECTION_LOCKED_TIME_MAX = 7;
int detetction_locked_time = DETECTION_LOCKED_TIME_MAX;

const int BACK_TO_ORIGINAL_TIME_MAX = 12;
int back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;

int get_pattern = 0;
int state_storage = 0;
int next_state = 0;

Car Moto_PWM;
Car Target_V;

uint8_t Motor_Flag = 0;

int32_t car_V;
sensor Car_sensor;
uint16_t Dis = 0.0;

float E_V = 0.0;

static float output;
float error;
float derivative;

int state = 0;
int state_storage = 0;
int next_state = 0;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
} PID_Controller;

PID_Controller pid = {500.0, 0.0, 0.0, 0.0, 0.0};
void Car_main(void)
{
    LED_Init();                 // ³õÊ¼»¯LED
    KEY_Init();                 // ³õÊ¼»¯°´¼ü
    OLED_Init();                // OLED³õÊ¼»¯
    OLED_CLS();                 // ÇåÆÁ
    uart_init(USART_2, 115200); // ³õÊ¼»¯´®¿Ú
    uart_init(USART_3, 115200); // ³õÊ¼»¯´®¿Ú
    Encoder_Init_TIM2();        // ±àÂëÆ÷³õÊ¼»¯
    Encoder_Init_TIM3();        // ±àÂëÆ÷³õÊ¼»¯
    Encoder_Init_TIM4();        // ±àÂëÆ÷³õÊ¼»¯
    MotorInit();
    sensor_init(); // µç»ú³õÊ¼»¯
    Ultrasonic_Init();

    Target_V.L = 0;
    Target_V.R = 0;
    Target_V.B = 0;

    while (1)
    {
        OLED_Task();
        Motor_Flag = 1;
    }
}

/**************************************************************
 * º¯ÊýÃû£ºcar_tim(void)
 * ¹¦  ÄÜ£º¶¨Ê±Æ÷ÈÎÎñÖ±ÐÐ±àÂëÆ÷¶ÁÈ¡
 * ×¢£º ¸Ãº¯ÊýÔÚ stm32f1xx_it.cÖÐ ÓÉµÎ´ð¶¨Ê±Æ÷ ÖÐ¶¨Ê±Ö´ÐÐ
 **************************************************************/
void car_tim(void)
{
    Dis = Get_Distance();
    if (Dis <= 15 && Dis != 1 && state != AVOIDING && state != ROTATE)
    {
        state = AVOIDING;
        rotate_time = ROTATE_TIME_MAX;
        back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
    }

    switch (state)
    {
    case RUN:
        Car_sensor.a = -(1 - Read_sensor(sensor2));
        Car_sensor.b = -(1 - Read_sensor(sensor3));
        Car_sensor.c = -(1 - Read_sensor(sensor4));
        Car_sensor.d = -(1 - Read_sensor(sensor1));

        E_V = (Car_sensor.a * 2 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 2);

        error = E_V;
        pid.integral += error;

        if (pid.integral > 1000)
            pid.integral = 1000;
        if (pid.integral < -1000)
            pid.integral = -1000;

        derivative = error - pid.previous_error;
        output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
        pid.previous_error = error;

        if (E_V == 0)
        {
            car_V = 800;
        }
        else
        {
            car_V = 0;
        }

        if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            next_state = MEET_FACE;
        }
        break;

    case MEET_FACE:
        next_state = SEARCH_EXIT_LEFT;
        left_rotate_times = LEFT_ROTATE_TIMES_MAX;
        right_rotate_times = RIGHT_ROTATE_TIMES_MAX;
        back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;
        dash_times = DASH_TIMES_MAX;
        break;

    case SEARCH_EXIT_LEFT:
        if (left_rotate_times > 0)
        {
            left_rotate_times--;
            car_V = 0;
            output = -800; // left rotate
            next_state = SEARCH_EXIT_LEFT;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            next_state = SEARCH_EXIT_RIGHT;
        }
        else
        {
            next_state = RUN;
        }
        break;

    case SEARCH_EXIT_RIGHT:
        if (right_rotate_times > 0)
        {
            right_rotate_times--;
            car_V = 0;
            output = 800; // right rotate
            next_state = SEARCH_EXIT_RIGHT;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            next_state = BACK_TO_FORWARD;
        }
        else
        {
            next_state = RUN;
        }
        break;

    case BACK_TO_FORWARD:
        if (back_to_forward_times > 0)
        {
            back_to_forward_times--;
            car_V = 0;
            output = -800; // right rotate
            next_state = BACK_TO_FORWARD;
        }
        else
        {
            next_state = DASH;
        }
        break;

    case DASH:
        if (dash_times > 0)
        {
            dash_times--;
            car_V = 1000;
            output = 0; // dash
            next_state = DASH;
        }
        else
        {
            next_state = RUN;
        }
        break;

    case AVOIDING:
        if (rotate_time > 0)
        {
            rotate_time--;
            Moto_PWM.L = 0;
            Moto_PWM.R = -800;
            Moto_PWM.B = 1600;
            next_state = AVOIDING;
        }
        else
        {
            rotate_time = ROTATE_TIME_MAX;
            back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
            next_state = ROTATE;
        }
        break;
    case ROTATE:
        if (back_to_original_time > 0)
        {
            back_to_original_time--;
            Moto_PWM.L = -800;
            Moto_PWM.R = -800;
            Moto_PWM.B = -800;
            next_state = ROTATE;
        }
        else
        {
            Moto_PWM.L = 0;
            Moto_PWM.R = 0;
            Moto_PWM.B = 0;
            next_state = RUN;
            back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
        }
        break;
    default:
        break;
    }
    state_storage = state;
    state = next_state;

    if(state != AVOIDING && state != ROTATE) {
        Moto_PWM.L = car_V + output;
        Moto_PWM.R = -car_V + output;
        Moto_PWM.B = output * 1.15;
    }

    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
}

// OLED ÏÔÊ¾ÄÚÈÝÈÎÎñº¯Êý
void OLED_Task(void)
{
    char txt[64];

    sprintf(txt, "PWM: L:%d R:%d B:%d ", Moto_PWM.L, Moto_PWM.R, Moto_PWM.B);
    OLED_P6x8Str(0, 2, txt); // ×Ö·û´®

    sprintf(txt, "state: %d", state_storage);
    OLED_P6x8Str(0, 3, txt); // ÏÔÊ¾×Ö·û´®

    Dis = Get_Distance();
    sprintf(txt, "Dis=%3d cm", Dis);
    OLED_P6x8Str(0, 4, txt); // ÏÔÊ¾×Ö·û´®

    
}
uint8_t Dis_falg = 0;
uint8_t read_flag = 0;
void Control(void) {}
