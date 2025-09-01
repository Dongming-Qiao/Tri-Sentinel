#include "Car_main.h"
#include "include.h"
#include "stdlib.h"

#define ALPHA 0.4f
static float filtered_dis_front = 0;    //filter for distance
static float filtered_dis_left = 0; 

//no use
#define FORWARD 0
#define BACKWARD 1
#define STOP 2
int state = STOP;
//no use end

float output = 0;

typedef enum {
    MAZE_FOLLOW_LEFT_WALL = 0,  // ?????
    MAZE_TURN_RIGHT,
		MAZE_TURN_LEFT // ??90?
} MazeState;

int l_actionnum = 0;
int r_actionnum = 0;

int forwardnum = 0;

MazeState maze_state = MAZE_FOLLOW_LEFT_WALL;
static uint16_t check_time = 0;
const uint32_t CHECK_INTERVAL = 20; // ?2???????

uint16_t rotate_time = 0;
const uint16_t ROTATE_TIME_90 = 8; 

const uint16_t WALL_DISTANCE = 10;    // ????(cm)
const uint16_t FRONT_DISTANCE = 15;   // ??????(cm)
const uint16_t LOST_WALL_DISTANCE = 15; // ??????(cm)
const uint16_t BIG_DISTANCE = 30;

CarSpeed Target_V = {0};
CarSpeed ENC_V = {0};
CarSpeed Moto_PWM = {0};
uint8_t Motor_Flag = 0;
int32_t car_V = 0;

SensorData Car_sensor = {0};

uint8_t Dis_falg = 0;
uint8_t read_flag = 0;

PID_Controller pid = {10.0, 0.0, 0.0, 0.0, 0.0};
float E_V = 0.0;

float prev_disl = 0.0;

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
    sensor_init(); 

    while (1)
    {
        OLED_Task();
        Motor_Flag = 1;
    }
}

void OLED_Task(void)
{   
	  
    char txt[64];
	
		prev_disl = Car_sensor.Dis_L;  
		Car_sensor.Dis=Get_Distance(1);
    Car_sensor.Dis_L=Get_Distance(2);
    low_pass_filter_f(Car_sensor.Dis);
		low_pass_filter_l(Car_sensor.Dis_L);
    sprintf(txt, "US_F:%3dcm", Car_sensor.Dis);
    OLED_P6x8Str(0, 1, txt);

    sprintf(txt, "US_L:%3dcm", Car_sensor.Dis_L);
    OLED_P6x8Str(0, 2, txt);

    sprintf(txt, "PWM:L%d R%d B%d", Moto_PWM.L, Moto_PWM.R, Moto_PWM.B);
    OLED_P6x8Str(0, 3, txt);
	
	sprintf(txt, "maze_state: %d", maze_state);
    OLED_P6x8Str(0, 4, txt);
}

float low_pass_filter_f(float new_dis) {
    filtered_dis_front = ALPHA * new_dis + (1 - ALPHA) * filtered_dis_front;
    return filtered_dis_front;
}

float low_pass_filter_l(float new_dis) {
    filtered_dis_left = ALPHA * new_dis + (1 - ALPHA) * filtered_dis_left;
    return filtered_dis_left;
}

void car_tim(void)
{
    check_time++;
    switch(state)
    {
        case FORWARD:
            car_V = 800;
            break;
        case BACKWARD:
            car_V = -600;
            break;
        case STOP:
            car_V = 0;
            break;
        default:
            car_V = 0;
            break;
    }

    switch(maze_state) {
        case MAZE_FOLLOW_LEFT_WALL:
						
						if (Car_sensor.Dis_L >= LOST_WALL_DISTANCE) {
								output = -100;
            } 
						else if (Car_sensor.Dis_L < WALL_DISTANCE) {
                output = 100;
            }
						else {
							output = 0;
						}
            
            state = FORWARD;
						
						if(Car_sensor.Dis_L >= BIG_DISTANCE){
                output = 0;
								maze_state = MAZE_TURN_LEFT;
								l_actionnum = 6;
								forwardnum = 20;
            }
						else if(Car_sensor.Dis < FRONT_DISTANCE || Car_sensor.Dis == 1)
						{
								maze_state = MAZE_TURN_RIGHT;
								r_actionnum = 5;
                state = STOP;
						}
						else {
                maze_state = MAZE_FOLLOW_LEFT_WALL;
            }
            
            break;
        case MAZE_TURN_RIGHT:
            state = STOP;
            output = 800; 
						if(r_actionnum != 0)
						{
							output = 800;
							maze_state = MAZE_TURN_RIGHT;
							r_actionnum--;
						}
						else
						{
							maze_state = MAZE_FOLLOW_LEFT_WALL;
						}
						
            break;
				case MAZE_TURN_LEFT:
						state = STOP;
						output = -800;
						if(l_actionnum != 0)
						{
							output = -800;
							maze_state = MAZE_TURN_LEFT;
							l_actionnum--;
						}
						else if(forwardnum != 0){
							car_V = 800;
							state = FORWARD;
							output = 0;
							forwardnum--;
						}
						else
						{
							maze_state = MAZE_FOLLOW_LEFT_WALL;
						}
						
						break;
        default:
            break;
    }

    // remove MotorFlag
    if (1)
    {
			if(maze_state == MAZE_FOLLOW_LEFT_WALL)
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
    else
    {
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
    }
    
    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L); 

    if(check_time % CHECK_INTERVAL == 0)
    {
        check_time = 0;
    }
}

void Control(void)
{
}
