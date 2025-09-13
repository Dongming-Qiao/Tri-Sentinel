/******************************************************************************
 * 文件功能：红外传感器数据串口发送程序
 * 说明：本程序专门用于将红外传感器检测到的数据通过串口通信形式发送
 *       不包含电机控制或其他功能，仅用于传感器数据调试和传输
 ******************************************************************************/

 #include "Car_main.h"
 #include "include.h"
 
 #define WAY_Flag 1        // 赛道选择标志，1:黑底白线，0:白底黑线
 
 // 传感器数据结构
 sensor Car_sensor;
 char sensor_info[7];      // 传感器信息发送缓冲区
 
 /*********************************************************************
  * 函数名：Car_main
  * 功能：系统主初始化函数
  * 参数：无
  * 返回值：无
  * 说明：初始化所有外设，程序仅用于传感器数据串口发送
  ********************************************************************/
 void Car_main(void)
 {
	 LED_Init();                     // 初始化LED
	 KEY_Init();                     // 初始化按键
	 OLED_Init();                    // OLED初始化
	 OLED_CLS();                     // 清屏
	 uart_init(USART_2, 115200);    // 初始化串口2
	 uart_init(USART_3, 115200);    // 初始化串口3（用于数据传输）
	 sensor_init();                  // 光电传感器初始化
 
	 while(1)
	 {
		 OLED_Task();                // 屏幕显示任务
	 }
 }
 
 /*********************************************************************
  * 函数名：car_tim
  * 功能：定时器中断服务函数，读取传感器数据并发送
  * 参数：无
  * 返回值：无
  * 说明：仅用于传感器数据采集和串口发送，不包含控制逻辑
  ********************************************************************/
 void car_tim(void)
 {
 #if WAY_Flag
	 // 黑底白线模式：直接读取传感器值
	 Car_sensor.a = Read_sensor(sensor1);
	 Car_sensor.b = Read_sensor(sensor2);
	 Car_sensor.c = Read_sensor(sensor3);
	 Car_sensor.d = Read_sensor(sensor4);
 #else
	 // 白底黑线模式：取反传感器值
	 Car_sensor.a = -Read_sensor(sensor1);
	 Car_sensor.b = -Read_sensor(sensor2);
	 Car_sensor.c = -Read_sensor(sensor3);
	 Car_sensor.d = -Read_sensor(sensor4);
 #endif
 
	 Send_Sensor_Data();  // 发送传感器数据
 }
 
 /*********************************************************************
  * 函数名：OLED_Task
  * 功能：OLED显示任务
  * 参数：无
  * 返回值：无
  * 说明：显示传感器原始数据，用于调试监控
  ********************************************************************/
 void OLED_Task(void)
 {
	 char txt[64];
	 
	 // 显示传感器原始数据
	 sprintf(txt, "Sensors: %d %d %d %d", 
			 Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d);
	 OLED_P6x8Str(0, 2, txt);
	 
	 // 显示数据发送状态
	 sprintf(txt, "UART3 Sending...");
	 OLED_P6x8Str(0, 3, txt);
	 
	 sprintf(txt, "Mode: %s", WAY_Flag ? "Black-White" : "White-Black");
	 OLED_P6x8Str(0, 4, txt);
	 
	 delay_ms(100);
 }
 
 /*********************************************************************
  * 函数名：get_info
  * 功能：传感器状态转换为字符
  * 参数：index - 传感器数值
  * 返回值：'1'（检测到）或 '0'（未检测到）
  * 说明：将模拟量转换为数字量表示
  ********************************************************************/
 char get_info(uint8_t index)
 {
	 return (index != 0) ? '1' : '0';
 }
 
 /*********************************************************************
  * 函数名：Send_Sensor_Data
  * 功能：通过串口发送传感器数据
  * 参数：无
  * 返回值：无
  * 说明：数据格式：4位传感器状态 + 模式标志 + 换行符
  ********************************************************************/
 void Send_Sensor_Data(void)
 {
	 // 转换传感器数据为字符格式
	 sensor_info[0] = get_info(Car_sensor.a);
	 sensor_info[1] = get_info(Car_sensor.b);
	 sensor_info[2] = get_info(Car_sensor.c);
	 sensor_info[3] = get_info(Car_sensor.d);
	 
 #if WAY_Flag
	 sensor_info[4] = '+';  // 黑底白线模式标志
 #else
	 sensor_info[4] = '-';  // 白底黑线模式标志
 #endif
 
	 sensor_info[5] = ' ';  // 分隔符
	 sensor_info[6] = '\n'; // 换行符（结束符）
 
	 // 通过USART3发送传感器数据
	 uart_SendBuf23(&USART3_Handler, (uint8_t*)sensor_info);
 }