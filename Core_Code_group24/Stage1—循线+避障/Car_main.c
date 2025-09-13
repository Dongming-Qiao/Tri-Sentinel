#include "Car_main.h"
#include "include.h"
#include <stdio.h>

// 全局变量声明
extern u8 ReadBuff[1024];
extern int ECPULSE1, ECPULSE2, ECPULSE3;
int speed = 2000;
uint8_t Dis_falg = 0;
uint8_t read_flag = 0;

// 车辆状态枚举定义
enum CarState
{
    RUN = 0,              // 正常运行状态
    MEET_FACE,            // 遇到障碍物
    BACKWARD,             // 后退状态
    SEARCH_EXIT_LEFT,     // 向左寻找出口
    SEARCH_EXIT_RIGHT,    // 向右寻找出口
    BACK_TO_FORWARD,      // 回到前进状态
    DASH,                 // 冲刺状态
    AVOIDING,             // 避障状态
    ROTATE                // 旋转状态
} Car_State;

// 状态计数器常量定义
const int BACKWARD_TIMES_MAX = 2;              // 后退状态最大次数
const int LEFT_ROTATE_TIMES_MAX = 6;           // 左旋转状态最大次数
const int RIGHT_ROTATE_TIMES_MAX = 12;         // 右旋转状态最大次数
const int ROTATE_BACKWARD_TIMES_MAX = 2;       // 旋转后退状态最大次数
const int BACK_TO_FORWARD_TIMES_MAX = 6;       // 回到前进状态最大次数
const int DASH_TIMES_MAX = 3;                  // 冲刺状态最大次数
const int ROTATE_TIME_MAX = 23;                // 旋转状态持续时间
const int DETECTION_LOCKED_TIME_MAX = 7;       // 锁定状态持续时间
const int BACK_TO_ORIGINAL_TIME_MAX = 13;      // 回到原点状态持续时间

// 状态计数器变量
int backward_times = BACKWARD_TIMES_MAX;                // 后退状态当前计数
int left_rotate_times = LEFT_ROTATE_TIMES_MAX;          // 左旋转状态当前计数
int right_rotate_times = RIGHT_ROTATE_TIMES_MAX;        // 右旋转状态当前计数
int rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;  // 旋转后退状态当前计数
int back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;  // 回到前进状态当前计数
int dash_times = DASH_TIMES_MAX;                        // 冲刺状态当前计数
int rotate_time = ROTATE_TIME_MAX;                      // 旋转状态当前时间
int detetction_locked_time = DETECTION_LOCKED_TIME_MAX; // 锁定状态当前时间
int back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;  // 回到原点状态当前时间

int get_pattern = 0;                                    // 模式获取标志

// 车辆控制结构体
Car Moto_PWM;                   // 电机PWM控制值
Car Target_V;                   // 目标速度值
uint8_t Motor_Flag = 0;         // 电机使能标志

// 控制变量
int32_t car_V;                  // 车辆速度
sensor Car_sensor;              // 传感器数据
uint16_t Dis = 0.0;             // 距离测量值
float E_V = 0.0;                // 误差值
static float output;            // PID输出值
float error;                    // 当前误差
float derivative;               // 微分项

// 状态机变量
int state = 0;                  // 当前状态
int state_storage = 0;          // 状态存储（用于显示）
int next_state = 0;             // 下一个状态

// PID控制器结构体
typedef struct
{
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float integral;         // 积分项
    float previous_error;   // 上一次误差
} PID_Controller;

// PID控制器初始化
PID_Controller pid = {500.0, 0.0, 0.0, 0.0, 0.0};

/******************************************************************************
 * 函数名：Car_main
 * 功能：车辆主程序，初始化所有外设并进入主循环
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void Car_main(void)
{
    // 初始化所有外设
    LED_Init();                 // 初始化LED
    KEY_Init();                 // 初始化按键
    OLED_Init();                // OLED初始化
    OLED_CLS();                 // 清屏
    uart_init(USART_2, 115200); // 初始化串口2
    uart_init(USART_3, 115200); // 初始化串口3
    Encoder_Init_TIM2();        // 编码器初始化
    Encoder_Init_TIM3();        // 编码器初始化
    Encoder_Init_TIM4();        // 编码器初始化
    MotorInit();                // 电机初始化
    sensor_init();              // 传感器初始化
    Ultrasonic_Init();          // 超声波初始化
    
    // 初始状态设置
    state = RUN;
    Target_V.L = 0;
    Target_V.R = 0;
    Target_V.B = 0;
    
    // 主循环
    while (1)
    {
        OLED_Task();    // OLED显示任务
        Motor_Flag = 1; // 电机使能标志
    }
}

/******************************************************************************
 * 函数名：car_tim
 * 功能：定时器中断服务函数，处理车辆状态机和运动控制
 * 参数：无
 * 返回值：无
 * 注：该函数在stm32f1xx_it.c中由定时器中断调用
 ******************************************************************************/
void car_tim(void)
{
    // 超声波避障检测
    if (Dis <= 9 && Dis != 1 && state != AVOIDING && state != ROTATE)
    {
        state = AVOIDING;
        rotate_time = ROTATE_TIME_MAX;
        back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
    }
    
    // 读取传感器数据
    Car_sensor.a = -(1 - Read_sensor(sensor2));
    Car_sensor.b = -(1 - Read_sensor(sensor3));
    Car_sensor.c = -(1 - Read_sensor(sensor4));
    Car_sensor.d = -(1 - Read_sensor(sensor1));

    // 状态机处理
    switch (state)
    {
    case RUN: // 正常运行状态
        E_V = (Car_sensor.a * 1.9 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 1.9);
        
        // 特殊情况处理
        if(Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == -1)
        {
            E_V = 0;
        }
        else if(Car_sensor.a == -1 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            E_V = 0;
        }
        
        // PID计算
        error = E_V;
        pid.integral += error;
        
        // 积分限幅
        if (pid.integral > 1000) pid.integral = 1000;
        if (pid.integral < -1000) pid.integral = -1000;

        derivative = error - pid.previous_error;
        output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
        pid.previous_error = error;

        // 速度控制
        if (E_V == 0)
        {
            car_V = 800; // 直行速度
        }
        else
        {
            car_V = 0;   // 转向时前进速度为0
        }

        // 检测到全黑线，切换到遇到障碍物状态
        if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            car_V = 0;
            next_state = MEET_FACE;
        }
        break;

    case MEET_FACE: // 遇到障碍物状态
        // 初始化所有计数器并切换到向左寻找出口状态
        next_state = SEARCH_EXIT_LEFT;
        left_rotate_times = LEFT_ROTATE_TIMES_MAX;
        right_rotate_times = RIGHT_ROTATE_TIMES_MAX;
        back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;
        dash_times = DASH_TIMES_MAX;
        break;
        
    case BACKWARD: // 后退状态
        if (backward_times > 0)
        {
            backward_times--;
            car_V = -800;   // 后退速度
            output = 0;     // 无转向
            next_state = SEARCH_EXIT_LEFT;
        }
        else
        {
            backward_times = BACKWARD_TIMES_MAX;
            next_state = SEARCH_EXIT_LEFT;
        }
        break;
        
    case SEARCH_EXIT_LEFT: // 向左寻找出口
        if (left_rotate_times > 0)
        {
            left_rotate_times--;
            car_V = 0;
            output = -800; // 左转
            next_state = SEARCH_EXIT_LEFT;
            rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            next_state = SEARCH_EXIT_RIGHT; // 仍然全黑，向右寻找
        }
        else if(rotate_backward_times > 0)
        {
            rotate_backward_times--;
            car_V = -800; // 后退
            output = 0;
            next_state = SEARCH_EXIT_LEFT;
        }
        else
        {
            next_state = RUN; // 找到出口，返回正常运行
        }
        break;

    case SEARCH_EXIT_RIGHT: // 向右寻找出口
        if (right_rotate_times > 0)
        {
            right_rotate_times--;
            car_V = 0;
            output = 800; // 右转
            next_state = SEARCH_EXIT_RIGHT;
            rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;
        }
        else if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            next_state = BACK_TO_FORWARD; // 仍然全黑，准备回到前进
        }
        else if(rotate_backward_times > 0)
        {
            rotate_backward_times--;
            car_V = -800; // 后退
            output = 0;
            next_state = SEARCH_EXIT_RIGHT;
        }
        else
        {
            next_state = RUN; // 找到出口，返回正常运行
        }
        break;

    case BACK_TO_FORWARD: // 回到前进状态
        if (back_to_forward_times > 0)
        {
            back_to_forward_times--;
            car_V = 0;
            output = -800; // 右转调整
            next_state = BACK_TO_FORWARD;
        }
        else
        {
            next_state = DASH; // 切换到冲刺状态
            back_to_forward_times = BACKWARD_TIMES_MAX;
        }
        break;

    case DASH: // 冲刺状态
        if (dash_times > 0)
        {
            dash_times--;
            car_V = 1000; // 高速前进
            output = 0;   // 无转向
            next_state = DASH;
        }
        else
        {
            next_state = RUN; // 返回正常运行
        }
        break;

    case AVOIDING: // 避障状态（超声波触发）
        if (rotate_time > 0)
        {
            rotate_time--;
            Moto_PWM.L = 0;
            Moto_PWM.R = -800;
            Moto_PWM.B = 1600;
            next_state = AVOIDING;
        }
        else
        {
            rotate_time = ROTATE_TIME_MAX;
            back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
            next_state = ROTATE;
        }
        break;
        
    case ROTATE: // 旋转状态
        if (back_to_original_time > 0)
        {
            back_to_original_time--;
            Moto_PWM.L = -800;
            Moto_PWM.R = -800;
            Moto_PWM.B = -800;
            next_state = ROTATE;
        }
        else
        {
            Moto_PWM.L = 0;
            Moto_PWM.R = 0;
            Moto_PWM.B = 0;
            next_state = RUN;
            back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
        }
        break;
        
    default:
        break;
    }
    
    // 保存当前状态并更新状态
    state_storage = state;
    state = next_state;

    // 计算电机PWM值（避障和旋转状态除外）
    if(state != AVOIDING && state != ROTATE) 
    {
        Moto_PWM.L = car_V + output;
        Moto_PWM.R = -car_V + output;
        Moto_PWM.B = output * 1.15;
    }

    // PWM值限幅
    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    // 执行电机控制
    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
}

/******************************************************************************
 * 函数名：OLED_Task
 * 功能：OLED显示任务，显示车辆状态信息
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void OLED_Task(void)
{
    char txt[64];

    // 显示电机PWM值
    sprintf(txt, "PWM: L:%d R:%d B:%d ", Moto_PWM.L, Moto_PWM.R, Moto_PWM.B);
    OLED_P6x8Str(0, 2, txt);

    // 显示当前状态
    sprintf(txt, "state: %d", state_storage);
    OLED_P6x8Str(0, 3, txt);

    // 显示超声波距离
    Dis = Get_Distance();
    sprintf(txt, "Dis=%3d cm", Dis);
    OLED_P6x8Str(0, 4, txt);
}

/******************************************************************************
 * 函数名：Control
 * 功能：预留控制函数
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void Control(void) 
{
    // 预留功能，可根据需要扩展
}