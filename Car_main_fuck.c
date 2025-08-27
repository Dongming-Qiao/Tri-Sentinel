#include "Car_main.h"
#include "include.h"
#include "stdlib.h"
#include "math.h"

#define WAY_Flag 0 // ÈüµÀÑ¡Ôñ£¬1:ºÚµ×°×Ïß 0:°×µ×ºÚÏß

//can shu
#define ALPHA 0.2f    // ?????????
#define MAX_ACCEL 100 // ???????

//state
#define STATE_NORMAL 0
#define STATE_TURNING 1
#define STATE_REVERSING 2

Car Target_V; // Ä¿±ê×ªËÙ
Car ENC_V; // Êµ¼Ê×ªËÙ
Car Moto_PWM; // ¿ØÖÆµç»ú¼ÆËã³öµÄPWM

uint8_t Motor_Flag = 0; // µç»úÆôÍ£±êÖ¾Î» 0£ºÍ£Ö¹ÔËÐÐ 1£º¿ªÊ¼ÔË¶¯
float E_V = 0.0;

int32_t car_V;
int32_t target_speed;
sensor Car_sensor;

int state = STATE_NORMAL;

const int32_t NORMAL_SPEED = 800;
const int32_t TURNING_SPEED = 0;
const int32_t REVERSE_SPEED = -600;

float E_V;
float error;
float derivative;
float output;

void update_speed_smoothly(void);
void determine_state(void);
void apply_motor_control(float steer_output);

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
    float max_integral; // ????
} PID_Controller;

// ??PID
PID_Controller steer_pid = {500.0, 0.0, 0.0, 0.0, 0.0, 1000.0};
// ??PID(??????)
PID_Controller speed_pid = {10.0, 0.5, 0.1, 0.0, 0.0, 500.0};

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
	
		target_speed = NORMAL_SPEED;
    car_V = 0;
    
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
	
		determine_state();
	
		switch(state)
    {
        case STATE_NORMAL:
            target_speed = NORMAL_SPEED;
            break;
        case STATE_TURNING:
            target_speed = TURNING_SPEED;
            break;
        case STATE_REVERSING:
            target_speed = REVERSE_SPEED;
            break;
    }
		
		update_speed_smoothly();
		
		E_V = (Car_sensor.a * 2 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 2);
    
    // PID??
    error = E_V;
    steer_pid.integral += error;
    
    // ????
    if (steer_pid.integral > steer_pid.max_integral) 
        steer_pid.integral = steer_pid.max_integral;
    if (steer_pid.integral < -steer_pid.max_integral) 
        steer_pid.integral = -steer_pid.max_integral;
    
		derivative = error - steer_pid.previous_error;
    output = steer_pid.Kp * error + steer_pid.Ki * steer_pid.integral + steer_pid.Kd * derivative;
    steer_pid.previous_error = error;
		
		apply_motor_control(output);
}

void determine_state(void)
{
#if WAY_Flag
    // ??????
    if (Car_sensor.a == 1 && Car_sensor.b == 1 && Car_sensor.c == 1 && Car_sensor.d == 1)
    {
        // ???????????,????
        state = STATE_REVERSING;
    }
    else if (Car_sensor.a == 1 || Car_sensor.b == 1 || Car_sensor.c == 1 || Car_sensor.d == 1)
    {
        // ????????????,????
        state = STATE_TURNING;
    }
    else
    {
        // ???????,????
        state = STATE_NORMAL;
    }
#else
    // ??????
    if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
    {
        // ???????????,????
        state = STATE_REVERSING;
    }
    else if (Car_sensor.a == 0 || Car_sensor.b == 0 || Car_sensor.c == 0 || Car_sensor.d == 0)
    {
        // ????????????,????
        state = STATE_TURNING;
    }
    else
    {
        // ???????,????
        state = STATE_NORMAL;
    }
#endif
}

void update_speed_smoothly(void)
{
		// ?????????????????
    static int32_t prev_speed = 0;
    int32_t speed_diff = target_speed - prev_speed;
    
    // ?????
    if (speed_diff > MAX_ACCEL) 
        speed_diff = MAX_ACCEL;
    else if (speed_diff < -MAX_ACCEL) 
        speed_diff = -MAX_ACCEL;
    
    car_V = prev_speed + speed_diff;
    prev_speed = car_V;
}

void apply_motor_control(float steer_output)
{
    // ???????????????PWM
    if (state == STATE_REVERSING)
    {
        // ???????
        Moto_PWM.L = car_V;
        Moto_PWM.R = car_V;
        Moto_PWM.B = car_V;
    }
    else
    {
        // ?????????
        Moto_PWM.L = car_V + steer_output;
        Moto_PWM.R = -car_V + steer_output;
        Moto_PWM.B = steer_output * 1.15f;
    }
    
    // PWM??
    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    // ??????
    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
}

void OLED_Task(void)
{
    char txt[64];
    
    // ??????????
    sprintf(txt, "S:%d%d%d%d", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d);
    OLED_P6x8Str(0, 2, txt);
    
    // ????
    const char* state_str;
    switch(state)
    {
        case STATE_NORMAL: state_str = "NORM"; break;
        case STATE_TURNING: state_str = "TURN"; break;
        case STATE_REVERSING: state_str = "REV"; break;
        default: state_str = "UNKN";
    }
    sprintf(txt, "State:%s Spd:%d", state_str, car_V);
    OLED_P6x8Str(0, 3, txt);
    
    // ??PWM?
    sprintf(txt, "PWM:L%5d R%5d", Moto_PWM.L, Moto_PWM.R);
    OLED_P6x8Str(0, 4, txt);
    sprintf(txt, "PWM:B%5d", Moto_PWM.B);
    OLED_P6x8Str(0, 5, txt);
    
    delay_ms(100);
}






