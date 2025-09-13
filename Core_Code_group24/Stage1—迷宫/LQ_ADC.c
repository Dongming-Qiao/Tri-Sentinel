// LQ_ADC.c
#include "include.h"
#include "LQ_ADC.h"

/******************************************************************************
 * ADC模块驱动程序
 * 功能：提供ADC采集、温度测量等功能
 ******************************************************************************/

ADC_HandleTypeDef ADC1_Handler;             // ADC句柄

/******************************************************************************
 * 函数名：ADC_Init
 * 功能：ADC初始化配置
 * 参数：无
 * 返回值：无
 * 说明：初始化ADC1，配置时钟、采样模式等参数
 ******************************************************************************/
void ADC_Init(void)
{ 
    RCC_PeriphCLKInitTypeDef ADC_CLKInit;
    
    // 配置ADC时钟
    ADC_CLKInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;    // ADC外设时钟
    ADC_CLKInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;       // 分频因子6，时钟为72M/6=12MHz
    HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);                 // 设置ADC时钟
    
    // 配置ADC参数
    ADC1_Handler.Instance = ADC1;
    ADC1_Handler.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // 右对齐
    ADC1_Handler.Init.ScanConvMode = DISABLE;                // 非扫描模式
    ADC1_Handler.Init.ContinuousConvMode = DISABLE;          // 关闭连续转换
    ADC1_Handler.Init.NbrOfConversion = 1;                   // 1个转换在规则序列中
    ADC1_Handler.Init.DiscontinuousConvMode = DISABLE;       // 禁止不连续采样模式
    ADC1_Handler.Init.NbrOfDiscConversion = 0;               // 不连续采样通道数为0
    ADC1_Handler.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 软件触发
    HAL_ADC_Init(&ADC1_Handler);                             // 初始化ADC
    
    HAL_ADCEx_Calibration_Start(&ADC1_Handler);              // 校准ADC
}

/******************************************************************************
 * 函数名：HAL_ADC_MspInit
 * 功能：ADC底层驱动初始化
 * 参数：hadc - ADC句柄
 * 返回值：无
 * 说明：配置ADC引脚和时钟，此函数会被HAL_ADC_Init()自动调用
 ******************************************************************************/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    // 使能时钟
    __HAL_RCC_ADC1_CLK_ENABLE();            // 使能ADC1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();           // 开启GPIOA时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();           // 开启GPIOB时钟
    
    // 配置GPIOA的ADC通道引脚
    GPIO_Initure.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // PA2, PA3
    GPIO_Initure.Mode = GPIO_MODE_ANALOG;         // 模拟模式
    GPIO_Initure.Pull = GPIO_NOPULL;              // 不带上下拉
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);
    
    // 配置GPIOB的ADC通道引脚
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // PB0, PB1
    GPIO_Initure.Mode = GPIO_MODE_ANALOG;         // 模拟模式
    GPIO_Initure.Pull = GPIO_NOPULL;              // 不带上下拉
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);
}

/******************************************************************************
 * 函数名：Get_Adc
 * 功能：获取指定通道的ADC转换值
 * 参数：ch - 通道编号（0~16，对应ADC_CHANNEL_0~ADC_CHANNEL_16）
 * 返回值：ADC转换结果（12位精度）
 ******************************************************************************/
uint16_t Get_Adc(uint32_t ch)
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;
    
    // 配置ADC通道
    ADC1_ChanConf.Channel = ch;                              // 通道选择
    ADC1_ChanConf.Rank = 1;                                  // 序列1
    ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // 采样时间
    HAL_ADC_ConfigChannel(&ADC1_Handler, &ADC1_ChanConf);    // 通道配置
    
    // 启动ADC转换并获取结果
    HAL_ADC_Start(&ADC1_Handler);                            // 开启ADC
    HAL_ADC_PollForConversion(&ADC1_Handler, 10);            // 轮询转换
    
    return (u16)HAL_ADC_GetValue(&ADC1_Handler);             // 返回转换结果
}

/******************************************************************************
 * 函数名：Get_Adc_Average
 * 功能：获取指定通道多次采样的平均值
 * 参数：ch - 通道编号
 *       times - 采样次数
 * 返回值：多次采样的平均值
 ******************************************************************************/
uint16_t Get_Adc_Average(uint32_t ch, uint8_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;
    
    for (t = 0; t < times; t++) {
        temp_val += Get_Adc(ch);
        delay_ms(5);  // 每次采样间隔5ms
    }
    return temp_val / times;
}

/******************************************************************************
 * 函数名：Get_Temp
 * 功能：根据ADC值计算温度
 * 参数：val - ADC采样值
 * 返回值：温度值（扩大了100倍，单位：℃）
 ******************************************************************************/
int Get_Temp(uint32_t val)
{
    int Temp;
    double temperate;
    
    // 计算电压值并转换为温度
    temperate = (float)val * (3.3 / 4096);          // 计算电压值
    temperate = (1.43 - temperate) / 0.0043 + 25;   // 转换为温度值
    Temp = temperate * 100;                         // 扩大100倍
    
    return Temp;
}

/******************************************************************************
 * 函数名：Test_ADC
 * 功能：ADC测试函数
 * 参数：无
 * 返回值：无
 * 说明：测试ADC采集功能并在OLED上显示结果
 ******************************************************************************/
void Test_ADC(void)
{
    char txt[32];
    uint32_t val1;
    int Temp;
    uint32_t Val4, Val5, Val6, Val7;
    
    // 初始化外设
    OLED_Init();    // OLED初始化
    OLED_CLS();     // 清屏
    ADC_Init();     // ADC初始化
    
    while (1) {
        // 读取各通道ADC值
        Val4 = Get_Adc(2);  // 通道2
        Val5 = Get_Adc(3);  // 通道3
        Val6 = Get_Adc(8);  // 通道8
        Val7 = Get_Adc(9);  // 通道9
        
        val1 = Val4;
        Temp = Get_Temp(val1);
        
        // 串口输出调试信息
        printf("Temp:%04d ADC4:%04d ADC5:%04d ADC6:%04d ADC7:%04d\n", 
               val1, Val4, Val5, Val6, Val7);
        
        // ADC值分级处理（示例）
        Val4 = (Val4 - 1000) / 600;
        if (Val4 < 1) {
            Val4 = 0;
        } else if (Val4 < 2) {
            Val4 = 1;
        } else if (Val4 < 3) {
            Val4 = 2;
        } else {
            Val4 = 3;
        }
        
        // OLED显示处理后的ADC值
        sprintf(txt, "ADC4:%04d", Val4);
        OLED_P6x8Str(0, 2, txt);
        
        delay_ms(100);  // 延时100ms
    }
}