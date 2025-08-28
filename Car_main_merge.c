#include "Car_main.h"
#include "include.h"
#include "stdlib.h"

// 全局变量定义
CarSpeed Target_V = {0};
CarSpeed ENC_V = {0};
CarSpeed Moto_PWM = {0};
uint8_t Motor_Flag = 0;
int32_t car_V = 0;

int speed = 2000;

SensorData Car_sensor = {0};

CarState Car_State = IDLE;

uint8_t Dis_falg = 0;
uint8_t read_flag = 0;

PID_Controller pid = {500.0, 0.0, 0.0, 0.0, 0.0};
float E_V = 0.0;

// infra module variables
int red_state = 0;
int red_state_storage = 0;
int red_next_state = 0;

enum
{
    RUN = 0,    
    MEET_FACE,
    SEARCH_EXIT_LEFT,
    SEARCH_EXIT_RIGHT,
    BACK_TO_FORWARD,
    DASH
} STATES;

const int LEFT_ROTATE_TIMES_MAX = 6;
int left_rotate_times = LEFT_ROTATE_TIMES_MAX;

const int RIGHT_ROTATE_TIMES_MAX = 12;
int right_rotate_times = RIGHT_ROTATE_TIMES_MAX;

const int BACK_TO_FORWARD_TIMES_MAX = 6;
int back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;

const int DASH_TIMES_MAX = 5;
int dash_times = DASH_TIMES_MAX;
// infra module variables end

//ultrasonic module variables
enum{
    RUN,
    AVOIDING,
    PATTERN_GOT_1,
    AVOIDING_AGAIN,
    PATTERN_GOT_2,
    BACK_TO_ORIGINAL
}Car_State;

int detetction_locked_time=0;
const int DETECTION_LOCKED_TIME_MAX=7;

int back_to_original_time=0;
const int BACK_TO_ORIGINAL_TIME_MAX=12;

const int TEST_TIMES = 8;
int test_times = TEST_TIMES;

int get_pattern=0;
int ultra_state_storage=0;
int ultra_next_state=0;

int ultra_state = RUN;
//ultrasonic module variables end


volatile uint8_t sensor_updated = 0;

// 主要的Car_main函数
void Car_main(void)
{
    LED_Init();
    KEY_Init();
    OLED_Init();
    OLED_CLS();
    uart_init(USART_2, 115200);
    uart_init(USART_3, 115200);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    MotorInit();
    Ultrasonic_Init();
    sensor_init(); // 红外传感器初始化

    Target_V.L = 0;
    Target_V.R = 0;
    Target_V.B = 0;

    Car_State = INFRARED_TRACKING;

    while (1)
    {
        OLED_Task();
        Motor_Flag = 1;
    }
}

// 定时器中断处理
void car_tim(void)
{
    Skip();
    if (Car_State == INFRARED_TRACKING)
    {
        infrared_motion();
    }
    else if (Car_State == ULTRASONIC_AVOID)
    {
        ultra_motion();
    }
    else
    {
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
        MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
    }
}

void Skip(void)
{
    if(ultra_state == RUN)
    {
        Car_State = INFRARED_TRACKING;
    }
    else if(ultra_state == AVOIDING || ultra_state == AVOIDING_AGAIN)
    {
        Car_State = ULTRASONIC_AVOID;
    }
}

// OLED显示任务
void OLED_Task(void)
{
    char txt[64];

    sprintf(txt, "State:%d", Car_State);
    OLED_P6x8Str(0, 1, txt);

    sprintf(txt, "IR:%d%d%d%d", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d);
    OLED_P6x8Str(0, 2, txt);

    sprintf(txt, "US:%3dcm E:%.1f", Car_sensor.Dis, E_V);
    OLED_P6x8Str(0, 3, txt);

    sprintf(txt, "PWM:L%d R%d B%d", Moto_PWM.L, Moto_PWM.R, Moto_PWM.B);
    OLED_P6x8Str(0, 4, txt);

    sprintf(txt, "Flag:%d", Motor_Flag);
    OLED_P6x8Str(0, 6, txt);
}

void ultra_motion(void)
{

    switch (ultra_state)
    {
    case RUN:
        if (Car_sensor.Dis > 15 || Car_sensor.Dis == 1)
        {
            Target_V.L = 1000;
            Target_V.R = -1000;
            Target_V.B = 0;
            ultra_next_state = RUN;
        }
        else
        {
            ultra_next_state = AVOIDING;
            get_pattern = 1;
        }
        break;
    case AVOIDING:
        // Avoid obstacle
        if (!get_pattern)
        {
            Target_V.L = 0;
            Target_V.R = -800;
            Target_V.B = 1600;
            ultra_next_state = AVOIDING;
        }
        else
        {
            Target_V.L = 0;
            Target_V.R = -800;
            Target_V.B = 1600;
            ultra_next_state = PATTERN_GOT_1;
        }
        break;
    case PATTERN_GOT_1:
        // First pattern detected
        if (detetction_locked_time > 0)
        {
            detetction_locked_time--;
            Target_V.L = 0;
            Target_V.R = -800;
            Target_V.B = 1600;
            ultra_next_state = PATTERN_GOT_1;
        }
        else
        {
            Target_V.L = 0;
            Target_V.R = 0;
            Target_V.B = 0;
            ultra_next_state = AVOIDING_AGAIN;
            detetction_locked_time = DETECTION_LOCKED_TIME_MAX;
        }
        break;
    case AVOIDING_AGAIN:
        // Detection locked
        if (!get_pattern)
        {
            Target_V.L = 0;
            Target_V.R = -800;
            Target_V.B = 1600;
            ultra_next_state = AVOIDING_AGAIN;
        }
        else
        {
            Target_V.L = 0;
            Target_V.R = -800;
            Target_V.B = 1600;
            ultra_next_state = PATTERN_GOT_2;
        }
        break;
    case PATTERN_GOT_2:
        // Second pattern detected
        Target_V.L = 0;
        Target_V.R = 0;
        Target_V.B = 0;
        ultra_next_state = BACK_TO_ORIGINAL;
        break;
    case BACK_TO_ORIGINAL:
        // Return to original state
        if (back_to_original_time > 0)
        {
            back_to_original_time--;
            Target_V.L = -800;
            Target_V.R = -800;
            Target_V.B = -800;
            ultra_next_state = BACK_TO_ORIGINAL;
        }
        else
        {
            Target_V.L = 0;
            Target_V.R = 0;
            Target_V.B = 0;
            ultra_next_state = RUN;
            back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
        }
        break;
    default:
        break;
    }
    ultra_state_storage = ultra_state;
    ultra_state = ultra_next_state;

    /* ENC_V.L=Read_Encoder(2);			//×óÂÖ
    ENC_V.R=Read_Encoder(4);			//ÓÒÂÖ
    ENC_V.B=Read_Encoder(3);			//ºóÂÖ */
    if (1)
    {
        // Moto_PWM.L += PidLocCtrl(&PID_L, Target_V.L - ENC_V.L);
        // Moto_PWM.R += PidLocCtrl(&PID_R, ENC_V.R - Target_V.R);
        // Moto_PWM.B += PidLocCtrl(&PID_B, Target_V.B - ENC_V.B);

        Moto_PWM.L = Target_V.L;
        Moto_PWM.R = Target_V.R;
        Moto_PWM.B = Target_V.B;
    }
    else
    {
        // ¹Ø±Õµç»úÊä³ö
        // Moto_PWM.L += PidLocCtrl(&PID_L, Target_V.L - ENC_V.L);
        // Moto_PWM.R += PidLocCtrl(&PID_R, ENC_V.R - Target_V.R);
        // Moto_PWM.B += PidLocCtrl(&PID_B, Target_V.B - ENC_V.B);
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
    }

    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
}


void infrared_motion(void)
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

    error = E_V;
    pid.integral += error;

    if (pid.integral > 1000)
        pid.integral = 1000;
    if (pid.integral < -1000)
        pid.integral = -1000;

    derivative = error - pid.previous_error;
    output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.previous_error = error;

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
    switch (red_state)
    {
    case RUN:
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
            red_next_state = MEET_FACE;
        }
        break;
    case MEET_FACE:
        red_next_state = SEARCH_EXIT_LEFT;
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
            red_next_state = SEARCH_EXIT_LEFT;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            red_next_state = SEARCH_EXIT_RIGHT;
        }
        else
        {
            red_next_state = RUN;
        }
        break;
    case SEARCH_EXIT_RIGHT:
        if (right_rotate_times > 0)
        {
            right_rotate_times--;
            car_V = 0;
            output = 800; // right rotate
            red_next_state = SEARCH_EXIT_RIGHT;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            red_next_state = BACK_TO_FORWARD;
        }
        else
        {
            red_next_state = RUN;
        }
        break;
    case BACK_TO_FORWARD:
        if (back_to_forward_times > 0)
        {
            back_to_forward_times--;
            car_V = 0;
            output = -800; // right rotate
            red_next_state = BACK_TO_FORWARD;
        }
        else
        {
            red_next_state = DASH;
        }
        break;
    case DASH:
        if (dash_times > 0)
        {
            dash_times--;
            car_V = 1000;
            output = 0; // dash
            red_next_state = DASH;
        }
        else
        {
            red_next_state = RUN;
        }
        break;
    default:
        break;
    }
    red_state_storage = red_state;
    red_state = red_next_state;

    
#endif

    // remove MotorFlag
    if (1)
    {
        if (abs(E_V) == 0)
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