#ifndef __CAR_MAIN_H__
#define __CAR_MAIN_H__

#include "include.h"
#include "LQ_PID.h"
#include "stdint.h"

// 外部变量声明
extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;

// 速度结构体
typedef struct {
    int L;
    int R;
    int B;
} CarSpeed;

// 传感器数据结构
typedef struct {
    uint16_t Dis;
    uint16_t Dis_L;
    uint16_t Dis_R;
    uint8_t a, b, c, d; // 红外传感器数据
} SensorData;

// 小车状态枚举
typedef enum {
    IDLE = 0,           // 空闲状态
    INFRARED_TRACKING,  // 红外循迹模式
    ULTRASONIC_AVOID,   // 超声波避障模式
} CarState;

// 全局变量声明
extern CarSpeed Target_V;
extern CarSpeed ENC_V;
extern CarSpeed Moto_PWM;
extern uint8_t Motor_Flag;
extern int32_t car_V;
extern SensorData Car_sensor;
extern CarState Car_State;
extern uint8_t Dis_falg;
extern uint8_t read_flag;

// PID控制器
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
} PID_Controller;

extern PID_Controller pid;
extern float E_V;

// 函数声明
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

#endif /* __CAR_MAIN_H__ */