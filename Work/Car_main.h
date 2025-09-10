#ifndef __CAR_MAIN_H__
#define __CAR_MAIN_H__

#include "include.h"
#include "LQ_PID.h"
#include "stdint.h"

// 外部变量声明
extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;

// 结构体定义
typedef struct
{
    int32_t L;  // 左轮速度/控制量
    int32_t R;  // 右轮速度/控制量
    int32_t B;  // 后轮速度/控制量
} Car;

typedef struct
{
    int8_t a;   // 传感器A数据
    int8_t b;   // 传感器B数据
    int8_t c;   // 传感器C数据
    int8_t d;   // 传感器D数据
} sensor;

// 全局变量声明
extern Car Target_V;        // 目标速度
extern Car ENC_V;           // 编码器速度
extern Car Moto_PWM;        // 电机PWM控制量
extern uint8_t Motor_Flag;  // 电机使能标志
extern int32_t car_V;       // 车辆速度
extern sensor Car_sensor;   // 传感器数据
extern uint8_t Dis_falg;    // 距离标志
extern uint8_t read_flag;   // 读取标志
extern int state_storage;   // 状态存储
extern int next_state;      // 下一个状态

// 主函数声明
void Car_main(void);        // 车辆主程序
void car_tim(void);         // 定时器控制函数
void OLED_Task(void);       // OLED显示任务

#endif /* __CAR_MAIN_H__ */