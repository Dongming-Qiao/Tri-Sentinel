// LQ_ADC.h
#ifndef __LQ_ADC_H
#define __LQ_ADC_H

#include "include.h"

// ADC函数声明
void ADC_Init(void);                        // ADC通道初始化
u16 Get_Adc(u32 ch);                        // 获取指定通道ADC值
u16 Get_Adc_Average(u32 ch, u8 times);      // 获取指定通道多次采样的平均值
void Test_ADC(void);                        // ADC测试函数
int Get_Temp(uint32_t val);                 // 根据ADC值计算温度

#endif