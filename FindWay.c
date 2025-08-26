#include "Car_main.h"
#include "include.h"
#include "stdlib.h"
#include <math.h>

#define WAY_Flag 0        // 赛道选择，1:黑底白线 0:白底黑线

//由Deepseek提供
// PID控制器结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float max_integral;   // 积分限幅
} PID_Controller;

Car Target_V; // 目标转速度
Car ENC_V;    // 实际转速度
Car Moto_PWM; // 控制电机计算出的PWM

uint8_t Motor_Flag = 0;   // 电机启停标志位 0：停止运行 1：开始运动
float E_V = 0.0;
int32_t car_V;
sensor Car_sensor;

// 新增变量
static PID_Controller line_pid = {180.0, 2.0, 25.0, 0.0, 0.0, 1000.0}; // PID参数
static int sensor_history[4][5] = {0}; // 传感器历史数据(4个传感器，5次历史)
static int history_index = 0;          // 历史数据索引
static int lost_line_counter = 0;      // 丢失线路计数器
static int last_valid_error = 0;       // 最后有效误差值

// 函数声明
float calculate_weighted_error(void);
void update_sensor_history(void);
int detect_line_status(void);
void apply_pid_control(void);
int constrain(int value, int min_val, int max_val);

void Car_main(void)
{
    LED_Init();             // 初始化LED    
    KEY_Init();             // 初始化按键
    OLED_Init();            // OLED初始化
    OLED_CLS();             // 清屏
    uart_init(USART_2,115200); // 初始化串口
    uart_init(USART_3,115200); // 初始化串口
    Encoder_Init_TIM2();    // 编码器初始化
    Encoder_Init_TIM3();    // 编码器初始化
    Encoder_Init_TIM4();    // 编码器初始化
    MotorInit();            // 电机初始化
    sensor_init();          // 光电传感器初始化
    
    while(1)
    {
        if (Read_key(KEY1) == 1)
        {
            if (Motor_Flag == 1)
                Motor_Flag = 0;
            else if (Motor_Flag == 0)
                Motor_Flag = 1;
        }
        OLED_Task();        // 屏幕显示        
    }
}

/*********************************************************************
* 函数名：void car_tim(void)
* 参数：无
* 返回值：无
* 功能：三轮小车 巡线程序
* 说明：提供两种赛道，白线黑底、黑线白底，WAY_Flag = 1:黑底白线 0:白底黑线
* 底色与线色差异大即可，具体可以调节光电灵敏度
********************************************************************/
void car_tim(void)
{
    // 读取并处理传感器数据
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

    // 更新传感器历史数据
    update_sensor_history();
    
    // 计算加权误差（使用历史数据平滑）
    E_V = calculate_weighted_error();
    
    // 检测线路状态
    int line_status = detect_line_status();
    
    // 根据线路状态调整速度
    switch(line_status) {
        case 0: // 正常巡线
            lost_line_counter = 0;
            // 根据误差大小动态调整速度
            float error_ratio = fabs(E_V) / 3.5f;
            car_V = 1000 + 400 * (1.0f - error_ratio);
            break;
            
        case 1: // 可能遇到十字路口
            lost_line_counter = 0;
            car_V = 1200; // 保持速度通过
            break;
            
        case 2: // 丢失线路
            lost_line_counter++;
            if(lost_line_counter > 10) {
                car_V = -800; // 长时间丢失线路则后退
            } else {
                // 使用最后有效误差继续前进
                car_V = 800;
                E_V = last_valid_error;
            }
            break;
    }
    
    // 保存有效误差值
    if(line_status != 2) {
        last_valid_error = E_V;
    }
    
    // 应用PID控制
    if (Motor_Flag) {
        apply_pid_control();
    } else {
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
    }
    
    // 输出限幅
    Moto_PWM.L = constrain(Moto_PWM.L, -6000, 6000);
    Moto_PWM.R = constrain(Moto_PWM.R, -6000, 6000);
    Moto_PWM.B = constrain(Moto_PWM.B, -6000, 6000);

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L); // 最终电机输出
}

// 计算加权误差（使用历史数据平滑）
float calculate_weighted_error(void)
{
    // 计算当前误差
    float current_error = (Car_sensor.a * 1.5 + Car_sensor.b * 0.5) - 
                         (Car_sensor.c * 0.5 + Car_sensor.d * 1.5);
    
    // 计算历史误差加权平均
    float historical_error = 0;
    float weight_sum = 0;
    
    for(int i = 0; i < 5; i++) {
        float weight = 1.0 / (i + 1); // 越近的数据权重越高
        historical_error += weight * (
            (sensor_history[0][i] * 1.5 + sensor_history[1][i] * 0.5) - 
            (sensor_history[2][i] * 0.5 + sensor_history[3][i] * 1.5)
        );
        weight_sum += weight;
    }
    
    historical_error /= weight_sum;
    
    // 混合当前误差和历史误差（70%当前，30%历史）
    return 0.7 * current_error + 0.3 * historical_error;
}

// 更新传感器历史数据
void update_sensor_history(void)
{
    sensor_history[0][history_index] = Car_sensor.a;
    sensor_history[1][history_index] = Car_sensor.b;
    sensor_history[2][history_index] = Car_sensor.c;
    sensor_history[3][history_index] = Car_sensor.d;
    
    history_index = (history_index + 1) % 5;
}

// 检测线路状态
int detect_line_status(void)
{
    // 检查是否所有传感器都检测到线（可能是十字路口）
    #if WAY_Flag
    if (Car_sensor.a == 1 && Car_sensor.b == 1 && Car_sensor.c == 1 && Car_sensor.d == 1)
    {
        return 1; // 十字路口
    }
    #else
    if (Car_sensor.a == -1 && Car_sensor.b == -1 && Car_sensor.c == -1 && Car_sensor.d == -1)
    {
        return 1; // 十字路口
    }
    #endif
    
    // 检查是否所有传感器都未检测到线（丢失线路）
    int active_sensors = 0;
    for(int i = 0; i < 4; i++) {
        if(sensor_history[i][history_index] != 0) {
            active_sensors++;
        }
    }
    
    if(active_sensors == 0) {
        return 2; // 丢失线路
    }
    
    return 0; // 正常巡线
}

// 应用PID控制
void apply_pid_control(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = GetSysTime(); // 需要实现获取系统时间的函数
    float dt = (current_time - last_time) / 1000.0f; // 转换为秒
    
    if(dt <= 0 || dt > 0.1f) {
        dt = 0.01f; // 默认10ms
    }
    
    last_time = current_time;
    
    // 计算PID
    line_pid.integral += E_V * dt;
    
    // 积分限幅
    if(line_pid.integral > line_pid.max_integral) {
        line_pid.integral = line_pid.max_integral;
    } else if(line_pid.integral < -line_pid.max_integral) {
        line_pid.integral = -line_pid.max_integral;
    }
    
    float derivative = (E_V - line_pid.prev_error) / dt;
    line_pid.prev_error = E_V;
    
    float pid_output = line_pid.Kp * E_V + line_pid.Ki * line_pid.integral + line_pid.Kd * derivative;
    
    // 应用PID输出到电机
    Moto_PWM.L = car_V + pid_output;
    Moto_PWM.R = -car_V + pid_output;
    Moto_PWM.B = pid_output * 1.2f;
}

// 约束函数
int constrain(int value, int min_val, int max_val) {
    if(value < min_val) return min_val;
    if(value > max_val) return max_val;
    return value;
}

void OLED_Task(void)
{
    char txt[64];
    
    // 屏幕显示
    sprintf(txt, "%d %d %d %d E:%.1f", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
    OLED_P6x8Str(0, 2, txt); // 字符串
    sprintf(txt, "ENC: L:%d R:%d B:%d ", ENC_V.L, ENC_V.R, ENC_V.B);
    OLED_P6x8Str(0, 3, txt); // 字符串
    sprintf(txt, "Tar: L:%d R:%d B:%d", Target_V.L, Target_V.R, Target_V.B);
    OLED_P6x8Str(0, 4, txt); // 字符串
    printf("samples:%d,%d,%d\n", ENC_V.L, ENC_V.R, ENC_V.B);
    delay_ms(100);
}