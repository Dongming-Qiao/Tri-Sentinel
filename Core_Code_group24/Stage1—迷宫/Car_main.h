#ifndef __CAR_MAIN_H__
#define __CAR_MAIN_H__

#include "include.h"
#include "stdint.h"

// 外部变量声明
extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;

// 车辆速度结构体
typedef struct {
    int L;  // 左轮速度
    int R;  // 右轮速度
    int B;  // 后轮速度
} CarSpeed;

// 传感器数据结构体
typedef struct {
    uint16_t Dis;      // 前方距离
    uint16_t Dis_L;    // 左侧距离
    uint16_t Dis_R;    // 右侧距离
} SensorData;

// PID控制器结构体
typedef struct {
    float Kp;              // 比例系数
    float Ki;              // 积分系数
    float Kd;              // 微分系数
    float integral;        // 积分项
    float previous_error;  // 上一次误差
} PID_Controller;

// 全局变量声明
extern CarSpeed Target_V;          // 目标速度
extern CarSpeed ENC_V;             // 编码器速度
extern CarSpeed Moto_PWM;          // 电机PWM值
extern uint8_t Motor_Flag;         // 电机使能标志
extern int32_t car_V;              // 车辆速度
extern SensorData Car_sensor;      // 传感器数据
extern uint8_t Dis_falg;           // 距离标志
extern uint8_t read_flag;          // 读取标志
extern PID_Controller pid;         // PID控制器
extern float E_V;                  // 误差值

// 函数声明
void Car_main(void);
void car_tim(void);
void OLED_Task(void);
void Control(void);

// 低通滤波器函数
float low_pass_filter_f(float new_dis);
float low_pass_filter_l(float new_dis);

#endif /* __CAR_MAIN_H__ */