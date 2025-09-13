#include "LQ_HCSR04.h"
#include "include.h"

/*
 * 接线说明：
 *   超声波模块与开发板用杜邦线相连
 *   传感器1：PB9接Trig, PB8接Echo
 *   传感器2：PB0接Trig, PB1接Echo
 *   GND接GND, 5V接VCC
 * 实验说明：
 *   下载程序，全速运行，安装OLED屏幕，屏幕显示距离值，说明实验成功
 */

/*-----------------------------------------------------------
 * 定时器初始化：初始化定时器2，定时时间为10us，用于超声波计时，系统主频72MHz
 *-----------------------------------------------------------*/

TIM_HandleTypeDef TIM2_Handler;      // 定时器句柄

/******************************************************************************
 * 函数名：LQ_TIM2_Init
 * 功能：定时器2初始化参数配置
 * 参数：arr - 自动重装载值
 *       psc - 预分频系数
 * 返回值：无
 * 说明：中断服务函数在 stm32f1xx_it.c 中
 ******************************************************************************/
void LQ_TIM2_Init(u16 arr, u16 psc)
{  
    __HAL_RCC_TIM2_CLK_ENABLE();                            // 使能TIM2时钟
    TIM2_Handler.Instance = TIM2;                           // 通用定时器2
    TIM2_Handler.Init.Prescaler = psc;                      // 分频系数
    TIM2_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;     // 向上计数器
    TIM2_Handler.Init.Period = arr;                         // 自动装载值
    TIM2_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler);   // 使能定时器2更新中断：TIM_IT_UPDATE   
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);  // 设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM2_IRQn);          // 开启ITM2中断  
}

/******************************************************************************
 * 函数名：Ultrasonic_Init
 * 功能：超声波模块初始化
 * 参数：无
 * 返回值：无
 * 说明：初始化超声波模块引脚配置
 ******************************************************************************/
void Ultrasonic_Init()
{
    GPIO_InitTypeDef GPIO_Initure;
  
    __HAL_RCC_GPIOB_CLK_ENABLE();           // 开启GPIOB时钟

    // 传感器1：TRIG1 - PB9输出
    GPIO_Initure.Pin = GPIO_PIN_9;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    
    // 传感器1：ECHO1 - PB8输入
    GPIO_Initure.Pin = GPIO_PIN_8;
    GPIO_Initure.Mode = GPIO_MODE_INPUT;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

    // 传感器2：TRIG2 - PB0输出
    GPIO_Initure.Pin = GPIO_PIN_0;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

    // 传感器2：ECHO2 - PB1输入
    GPIO_Initure.Pin = GPIO_PIN_1;
    GPIO_Initure.Mode = GPIO_MODE_INPUT;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_Initure); 

    // 初始化Trig引脚为低电平
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

// 用于超声波计时的外部变量，位于stm32f1xx_it.c中
extern uint32_t TimeCounter;

/******************************************************************************
 * 函数名：Get_Distance
 * 功能：获取超声波传感器距离
 * 参数：sensor_num - 传感器编号（1或2）
 * 返回值：距离值（单位：厘米）
 * 说明：传入时间单位为10us
 ******************************************************************************/
uint32_t Get_Distance(uint8_t sensor_num)
{
    uint32_t Distance = 0;
    uint32_t HalTime1 = 0, HalTime2 = 0;
    uint16_t trig_pin, echo_pin;
    GPIO_TypeDef* trig_port, *echo_port;

    // 选择传感器引脚
    if (sensor_num == 1) {
        // 传感器1: PB9=TRIG, PB8=ECHO
        trig_pin = GPIO_PIN_9;
        trig_port = GPIOB;
        echo_pin = GPIO_PIN_8;
        echo_port = GPIOB;
    } else {
        // 传感器2: PB0=TRIG, PB1=ECHO
        trig_pin = GPIO_PIN_0;
        trig_port = GPIOB;
        echo_pin = GPIO_PIN_1;
        echo_port = GPIOB;
    }

    /* 将Trig引脚拉高10微秒 */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    
    // 等待Echo引脚变为高电平（接收到返回信号）
    HalTime1 = TimeCounter;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == 0) {
        if (TimeCounter - HalTime1 > 2200) break;  // 超时保护：10us * 2200
    }
    
    // 记录高电平开始时间
    HalTime1 = TimeCounter;
    // 等待高电平结束
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == 1) {
        if (TimeCounter - HalTime1 > 4500) break;  // 超时保护
    }
    
    // 计算高电平持续时间
    if (TimeCounter > HalTime1) {
        HalTime2 = TimeCounter - HalTime1;
        // 距离计算：时间(us) * 声速(340m/s) / 2 / 10000
        Distance = (uint32_t)(((float)HalTime2 * 17) / 100);
    }
    
    // 返回有效距离或1（表示无效测量）
    if (Distance > 2)
        return Distance;
    else 
        return 1;
}

/******************************************************************************
 * 函数名：Test_Ultrasonic
 * 功能：超声波测试函数
 * 参数：无
 * 返回值：无
 * 说明：初始化超声波模块并在OLED上显示距离值
 ******************************************************************************/
void Test_Ultrasonic()
{
    uint16_t Dis1 = 0;
    uint16_t Dis2 = 0;
    char txt[32];
    
    Ultrasonic_Init();
    OLED_Init();                        // OLED初始化
    OLED_P6x8Str(10, 0, "Test HCSR04"); // 显示标题
    
    delay_ms(5);
    
    while (1) {
        Dis1 = Get_Distance(1);         // 获取传感器1距离
        Dis2 = Get_Distance(2);         // 获取传感器2距离
        
        // 显示距离值
        sprintf(txt, "Dis1=%3d cm", Dis1);
        OLED_P8x16Str(10, 2, txt);
        
        sprintf(txt, "Dis2=%3d cm", Dis2);
        OLED_P8x16Str(10, 4, txt);
    
        LED_Ctrl(RVS);                  // 控制LED状态
        delay_ms(100);                  // 延时100ms
    }
}