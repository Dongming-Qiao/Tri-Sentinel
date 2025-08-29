#ifndef __CAR_MAIN_H__
#define __CAR_MAIN_H__

#include "include.h"
#include "LQ_PID.h"
#include "stdint.h"

// ??????
extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;

// ?????
typedef struct
{
    int32_t L;
    int32_t R;
    int32_t B;
} Car;


typedef struct
{
    int8_t a;
    int8_t b;
    int8_t c;
    int8_t d;
} sensor;


// ??????
extern Car Target_V;
extern Car ENC_V;
extern Car Moto_PWM;
extern uint8_t Motor_Flag;
extern int32_t car_V;
extern sensor Car_sensor;
extern uint8_t Dis_falg;
extern uint8_t read_flag;
extern int state_storage;
extern int next_state;

// ????
void Car_main(void);
void car_tim(void);
void OLED_Task(void);
void Control(void);

// ??????
void Infrared_Init(void);
void Infrared_TrackingTask(void);
uint8_t Infrared_IsTapeObstacle(void);
uint8_t Infrared_IsPathLost(void);

// ???????
void Ultrasonic_Init(void);
uint16_t Ultrasonic_GetDistance(void);
void Ultrasonic_AvoidanceTask(void);

#endif /* __CAR_MAIN_H__ */
