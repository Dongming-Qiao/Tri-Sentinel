#include "Car_main.h"
#include "include.h"
#include "stdlib.h"

#define ALPHA 0.4f                    // 低通滤波器系数

// 滤波器状态变量
static float filtered_dis_front = 0;  // 前方距离滤波值
static float filtered_dis_left = 0;   // 左侧距离滤波值

// 迷宫状态枚举
typedef enum {
    MAZE_FOLLOW_LEFT_WALL = 0,        // 沿左墙行走
    MAZE_TURN_RIGHT,                  // 右转
    MAZE_TURN_LEFT                    // 左转
} MazeState;

#define FORWARD 0
#define BACKWARD 1
#define STOP 2
int state = STOP;

// 动作计数器
int l_actionnum = 0;                  // 左转动作计数
int r_actionnum = 0;                  // 右转动作计数
int forwardnum = 0;                   // 前进动作计数

// 状态变量
MazeState maze_state = MAZE_FOLLOW_LEFT_WALL;  // 当前迷宫状态
static uint16_t check_time = 0;                // 检查计时器
const uint32_t CHECK_INTERVAL = 20;            // 检查间隔（20ms）

// 旋转时间常量
const uint16_t ROTATE_TIME_90 = 8;             // 90度旋转时间

// 距离阈值常量
const uint16_t WALL_DISTANCE = 15;             // 墙壁距离阈值(cm)
const uint16_t FRONT_DISTANCE = 15;            // 前方障碍距离阈值(cm)
const uint16_t LOST_WALL_DISTANCE = 16;        // 丢失墙壁距离阈值(cm)
const uint16_t BIG_DISTANCE = 40;              // 大距离阈值(cm)

// 控制变量
CarSpeed Target_V = {0};              // 目标速度
CarSpeed ENC_V = {0};                 // 编码器速度
CarSpeed Moto_PWM = {0};              // 电机PWM值
uint8_t Motor_Flag = 0;               // 电机使能标志
int32_t car_V = 0;                    // 车辆速度
SensorData Car_sensor = {0};          // 传感器数据
uint8_t Dis_falg = 0;                 // 距离标志
uint8_t read_flag = 0;                // 读取标志

// PID控制器
PID_Controller pid = {10.0, 0.0, 0.0, 0.0, 0.0};
float E_V = 0.0;                      // 误差值
float output = 0;                     // 控制输出

// 历史距离记录
float prev_disl = 0.0;                // 上一次左侧距离

/******************************************************************************
 * 函数名：Car_main
 * 功能：车辆主程序，初始化所有外设
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void Car_main(void)
{
    // 初始化所有外设
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

    // 主循环
    while (1)
    {
        OLED_Task();      // OLED显示任务
        Motor_Flag = 1;   // 电机使能
    }
}

/******************************************************************************
 * 函数名：OLED_Task
 * 功能：OLED显示任务，显示传感器数据和状态信息
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void OLED_Task(void)
{
    char txt[64];
    
    // 获取并滤波距离数据
    prev_disl = Car_sensor.Dis_L;
    Car_sensor.Dis = Get_Distance(1);     // 获取前方距离
    Car_sensor.Dis_L = Get_Distance(2);   // 获取左侧距离
    low_pass_filter_f(Car_sensor.Dis);    // 前方距离滤波
    low_pass_filter_l(Car_sensor.Dis_L);  // 左侧距离滤波
    
    // 显示前方距离
    sprintf(txt, "US_F:%3dcm", Car_sensor.Dis);
    OLED_P6x8Str(0, 1, txt);
    
    // 显示左侧距离
    sprintf(txt, "US_L:%3dcm", Car_sensor.Dis_L);
    OLED_P6x8Str(0, 2, txt);
    
    // 显示电机PWM值
    sprintf(txt, "PWM:L%d R%d B%d", Moto_PWM.L, Moto_PWM.R, Moto_PWM.B);
    OLED_P6x8Str(0, 3, txt);
    
    // 显示迷宫状态
    sprintf(txt, "maze_state: %d", maze_state);
    OLED_P6x8Str(0, 4, txt);
}

/******************************************************************************
 * 函数名：low_pass_filter_f
 * 功能：前方距离低通滤波器
 * 参数：new_dis - 新的距离测量值
 * 返回值：滤波后的距离值
 ******************************************************************************/
float low_pass_filter_f(float new_dis)
{
    filtered_dis_front = ALPHA * new_dis + (1 - ALPHA) * filtered_dis_front;
    return filtered_dis_front;
}

/******************************************************************************
 * 函数名：low_pass_filter_l
 * 功能：左侧距离低通滤波器
 * 参数：new_dis - 新的距离测量值
 * 返回值：滤波后的距离值
 ******************************************************************************/
float low_pass_filter_l(float new_dis)
{
    filtered_dis_left = ALPHA * new_dis + (1 - ALPHA) * filtered_dis_left;
    return filtered_dis_left;
}

/******************************************************************************
 * 函数名：car_tim
 * 功能：定时器中断服务函数，处理迷宫导航状态机
 * 参数：无
 * 返回值：无
 ******************************************************************************/
void car_tim(void)
{
    check_time++;
    
    // 迷宫状态机处理
    switch(maze_state)
    {
        case MAZE_FOLLOW_LEFT_WALL:  // 沿左墙行走状态
            // 左侧距离控制
            if (Car_sensor.Dis_L >= LOST_WALL_DISTANCE)
            {
                output = -100;  // 距离过远，向右修正
            }
            else if (Car_sensor.Dis_L < WALL_DISTANCE)
            {
                output = 100;   // 距离过近，向左修正
            }
            else
            {
                output = 0;     // 保持距离
            }
            
            car_V = 800;        // 前进
            
            // 状态转换判断
            if (Car_sensor.Dis_L >= BIG_DISTANCE)
            {
                // 发现大空间，准备左转
                output = 0;
                maze_state = MAZE_TURN_LEFT;
                l_actionnum = 5;
                forwardnum = 20;
            }
            else if (Car_sensor.Dis < FRONT_DISTANCE || Car_sensor.Dis == 1)
            {
                // 前方有障碍，准备右转
                maze_state = MAZE_TURN_RIGHT;
                r_actionnum = 7;
                car_V = 0;
            }
            break;
            
        case MAZE_TURN_RIGHT:  // 右转状态
            car_V = 0;          // 停止前进
            if (r_actionnum != 0)
            {
                // 执行右转动作
                output = 800;
                r_actionnum--;
            }
            else
            {
                // 右转完成，返回沿墙行走
                maze_state = MAZE_FOLLOW_LEFT_WALL;
            }
            break;
            
        case MAZE_TURN_LEFT:   // 左转状态
            car_V = 0;         // 停止前进
            if (l_actionnum != 0)
            {
                // 执行左转动作
                output = -800;
                l_actionnum--;
            }
            else if (forwardnum != 0)
            {
                // 左转后前进一段距离
                car_V = 800;
                output = 0;
                forwardnum--;
            }
            else
            {
                // 左转完成，返回沿墙行走
                maze_state = MAZE_FOLLOW_LEFT_WALL;
            }
            break;
            
        default:
            break;
    }
    
    // 电机控制计算
    if (maze_state == MAZE_FOLLOW_LEFT_WALL)
    {
        // 沿墙行走模式控制
        Moto_PWM.L = car_V + output;
        Moto_PWM.R = -car_V + output;
        Moto_PWM.B = output * 1.15;
    }
    else
    {
        // 转向模式控制
        Moto_PWM.L = car_V + output;
        Moto_PWM.R = -car_V + output;
        Moto_PWM.B = output;
    }
    
    // PWM值限幅
    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));
    
    // 执行电机控制
    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);
    
    // 定时检查重置
    if (check_time % CHECK_INTERVAL == 0)
    {
        check_time = 0;
    }
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
