#ifndef __CAR_MAIN_H__
#define __CAR_MAIN_H__

#include "include.h"
#include "stdint.h"

extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;

typedef struct {
    int L;
    int R;
    int B;
} CarSpeed;

typedef struct {
    uint16_t Dis;
    uint16_t Dis_L;
    uint16_t Dis_R;
    uint8_t a, b, c, d; // Infrared sensor data
} SensorData;

extern CarSpeed Target_V;
extern CarSpeed ENC_V;
extern CarSpeed Moto_PWM;
extern uint8_t Motor_Flag;
extern int32_t car_V;

extern int32_t car_rotate_angle;  //rotate angle relative to start point

extern SensorData Car_sensor;

extern uint8_t Dis_falg;
extern uint8_t read_flag;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
} PID_Controller;

extern PID_Controller pid;
extern float E_V;

void Car_main(void);
void car_tim(void);
void OLED_Task(void);
void Control(void);

// 红外相关函数
void Infrared_Init(void);
void Infrared_TrackingTask(void);
uint8_t Infrared_IsTapeObstacle(void);
uint8_t Infrared_IsPathLost(void);

// 超声波相关函数
void Ultrasonic_Init(void);
uint16_t Ultrasonic_GetDistance(void);
void Ultrasonic_AvoidanceTask(void);

void Switch(void);
void infrared_motion(void);
void ultra_motion(void);

#endif /* __CAR_MAIN_H__ */