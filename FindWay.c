#include "Car_main.h"
#include "include.h"
#include "stdlib.h"

#define WAY_Flag 0        // 赛道选择，1:黑底白线 0:白底黑线

// PID控制器结构体
typedef struct {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    int32_t integral;
    int32_t prev_error;
    int32_t max_integral;   // 积分限幅
} PID_Controller;

Car Target_V;
Car ENC_V;
Car Moto_PWM;
uint8_t Motor_Flag = 0;
int32_t E_V = 0;
int32_t car_V;
sensor Car_sensor;

// 新增变量 - 使用整数运算替代浮点
static PID_Controller line_pid = {180, 2, 25, 0, 0, 1000}; // PID参数(放大100倍)
static int32_t sensor_history[4][5] = {0}; // 传感器历史数据
static uint8_t history_index = 0;
static uint8_t lost_line_counter = 0;
static int32_t last_valid_error = 0;

// 函数声明
int32_t calculate_weighted_error(void);
void update_sensor_history(void);
uint8_t detect_line_status(void);
void apply_pid_control(void);
int32_t abs_int(int32_t value);
int32_t constrain(int32_t value, int32_t min_val, int32_t max_val);

// 简单的绝对值函数
int32_t abs_int(int32_t value) {
    return (value < 0) ? -value : value;
}

void Car_main(void)
{
    // 初始化代码保持不变...
}

void car_tim(void)
{
    // 读取传感器数据
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
    
    // 计算加权误差
    E_V = calculate_weighted_error();
    
    // 检测线路状态
    uint8_t line_status = detect_line_status();
    
    // 根据线路状态调整速度
    switch(line_status) {
        case 0: // 正常巡线
            lost_line_counter = 0;
            // 整数运算替代浮点：error_ratio = |E_V| * 100 / 350
            int32_t error_ratio = (abs_int(E_V) * 100) / 350;
            car_V = 1000 + (400 * (100 - error_ratio)) / 100;
            break;
            
        case 1: // 十字路口
            lost_line_counter = 0;
            car_V = 1200;
            break;
            
        case 2: // 丢失线路
            lost_line_counter++;
            if(lost_line_counter > 10) {
                car_V = -800;
            } else {
                car_V = 800;
                E_V = last_valid_error;
            }
            break;
    }
    
    // 保存有效误差值
    if(line_status != 2) {
        last_valid_error = E_V;
    }
    
    // 应用控制
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

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
}

// 计算加权误差（整数运算版本）
int32_t calculate_weighted_error(void)
{
    // 计算当前误差（放大100倍）
    int32_t current_error = (Car_sensor.a * 150 + Car_sensor.b * 50) - 
                           (Car_sensor.c * 50 + Car_sensor.d * 150);
    
    // 计算历史误差加权平均
    int32_t historical_error = 0;
    int32_t weight_sum = 0;
    
    for(uint8_t i = 0; i < 5; i++) {
        int32_t weight = 100 / (i + 1); // 权重放大100倍
        historical_error += weight * (
            (sensor_history[0][i] * 150 + sensor_history[1][i] * 50) - 
            (sensor_history[2][i] * 50 + sensor_history[3][i] * 150)
        );
        weight_sum += weight;
    }
    
    historical_error = (historical_error * 100) / weight_sum; // 除以权重和
    
    // 混合当前误差和历史误差（70%当前，30%历史）
    return (70 * current_error + 30 * historical_error) / 100;
}

void update_sensor_history(void)
{
    sensor_history[0][history_index] = Car_sensor.a;
    sensor_history[1][history_index] = Car_sensor.b;
    sensor_history[2][history_index] = Car_sensor.c;
    sensor_history[3][history_index] = Car_sensor.d;
    
    history_index = (history_index + 1) % 5;
}

uint8_t detect_line_status(void)
{
    #if WAY_Flag
    if (Car_sensor.a == 1 && Car_sensor.b == 1 && Car_sensor.c == 1 && Car_sensor.d == 1)
    {
        return 1;
    }
    #else
    if (Car_sensor.a == -1 && Car_sensor.b == -1 && Car_sensor.c == -1 && Car_sensor.d == -1)
    {
        return 1;
    }
    #endif
    
    // 检查是否所有传感器都未检测到线
    uint8_t active_sensors = 0;
    for(uint8_t i = 0; i < 4; i++) {
        if(sensor_history[i][history_index] != 0) {
            active_sensors++;
        }
    }
    
    if(active_sensors == 0) {
        return 2;
    }
    
    return 0;
}

void apply_pid_control(void)
{
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick(); // 使用HAL库获取时间
    int32_t dt = (current_time - last_time);
    
    if(dt <= 0 || dt > 100) {
        dt = 10; // 默认10ms
    }
    
    last_time = current_time;
    
    // 计算PID（使用整数运算）
    line_pid.integral += (E_V * dt) / 10; // 积分项
    
    // 积分限幅
    if(line_pid.integral > line_pid.max_integral) {
        line_pid.integral = line_pid.max_integral;
    } else if(line_pid.integral < -line_pid.max_integral) {
        line_pid.integral = -line_pid.max_integral;
    }
    
    int32_t derivative = ((E_V - line_pid.prev_error) * 100) / dt; // 微分项
    line_pid.prev_error = E_V;
    
    int32_t pid_output = (line_pid.Kp * E_V + line_pid.Ki * line_pid.integral + line_pid.Kd * derivative) / 100;
    
    // 应用PID输出到电机
    Moto_PWM.L = car_V + pid_output;
    Moto_PWM.R = -car_V + pid_output;
    Moto_PWM.B = (pid_output * 120) / 100; // *1.2
}

int32_t constrain(int32_t value, int32_t min_val, int32_t max_val) {
    if(value < min_val) return min_val;
    if(value > max_val) return max_val;
    return value;
}

void OLED_Task(void)
{
    char txt[64];
    sprintf(txt, "%d %d %d %d E:%d", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
    OLED_P6x8Str(0, 2, txt);
    // 其他显示代码...
}
