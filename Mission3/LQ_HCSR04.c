#include "LQ_HCSR04.h"
#include "include.h"
/*
1.½ÓÏß:
	  ³¬Éù²¨Ä£¿éÓë¿ª·¢°åÓÃ¶Å°îÏßÏàÁ¬£¬°å×ÓPB9½ÓTrig, PB8½ÓEcho, GND½ÓGND 5V½ÓVCC¡£
2.ÊµÑé:
	  ÏÂÔØ³ÌÐò£¬È«ËÙÔËÐÐ£¬°²×°OLEDÆÁÄ»£¬ÆÁÄ»ÏÔÊ¾¾àÀëÖµ¡£ËµÃ÷ÊµÑé³É¹¦
*/
/*-----------------------------------------------------------
¶¨Ê±Æ÷³õÊ¼»¯ ³õÊ¼»¯¶¨Ê±Æ÷2 ¶¨Ê±Ê±¼äÎª10us ÓÃÀ´¸ø³¬Éù²¨¼ÆÊ±£¬ÏµÍ³Ö÷Æµ72MHz

-----------------------------------------------------------*/
/**************************************************
 * º¯ÊýÃû: TIM2_Init(u16 arr,u16 psc)
 * ¹¦  ÄÜ: ¶¨Ê±Æ÷2³õÊ¼»¯²ÎÊýÅäÖÃ
 * ·µ»ØÖµ: ×°ÔØÖµarr£¬Ô¤·ÖÆµÏµÊýÖµpsc
 * /ÖÐ¶Ï·þÎñº¯ÊýÔÚ stm32f1xx_it.cÖÐ
 ***************************************************/
 
TIM_HandleTypeDef TIM2_Handler;      //¶¨Ê±Æ÷¾ä±ú 

void LQ_TIM2_Init(u16 arr,u16 psc)
{  
    __HAL_RCC_TIM2_CLK_ENABLE();                            //Ê¹ÄÜTIM3Ê±ÖÓ
    TIM2_Handler.Instance=TIM2;                             //Í¨ÓÃ¶¨Ê±Æ÷3
    TIM2_Handler.Init.Prescaler=psc;                        //·ÖÆµÏµÊý
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;       //ÏòÉÏ¼ÆÊýÆ÷
    TIM2_Handler.Init.Period=arr;                           //×Ô¶¯×°ÔØÖµ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; //Ê±ÖÓ·ÖÆµÒò×Ó
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler);   //Ê¹ÄÜ¶¨Ê±Æ÷3ºÍ¶¨Ê±Æ÷2¸üÐÂÖÐ¶Ï£ºTIM_IT_UPDATE   
	HAL_NVIC_SetPriority(TIM2_IRQn,0,1);    //ÉèÖÃÖÐ¶ÏÓÅÏÈ¼¶£¬ÇÀÕ¼ÓÅÏÈ¼¶1£¬×ÓÓÅÏÈ¼¶3
	HAL_NVIC_EnableIRQ(TIM2_IRQn);          //¿ªÆôITM2ÖÐ¶Ï  
}

void Ultrasonic_Init()
{
	//³õÊ¼»¯ ³¬Éù²¨Ä£¿éÒý½Å
    GPIO_InitTypeDef GPIO_Initure;
  
	__HAL_RCC_GPIOB_CLK_ENABLE();                       //¿ªÆôGPIOBÊ±ÖÓ

	GPIO_Initure.Pin = GPIO_PIN_4;   // TRIG1 - PA4输出
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

	GPIO_Initure.Pin = GPIO_PIN_5;   // ECHO1 - PA5输入
    GPIO_Initure.Mode = GPIO_MODE_INPUT;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure); 

    GPIO_Initure.Pin=GPIO_PIN_9;  	                    //PB9 // Trig - PB9Êä³ö
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      	    //Êä³ö
    GPIO_Initure.Pull=GPIO_PULLUP;        			    //ÉÏÀ­
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//¸ßËÙ
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);    
	
	GPIO_Initure.Pin=GPIO_PIN_8;                       //PB8 // Echo - PB8Êä
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//ÊäÈë
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//ÏÂÀ­
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;            //¸ßËÙ
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 


    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	LQ_TIM2_Init(71,9);                    // ¶¨Ê±Æ÷ÓÃÓÚ¼ÆÊý
}
extern uint32_t TimeCounter;	 //ÓÃÓÚ³¬Éù²¨¼ÆÊ±,¸Ã±äÁ¿Î»ÓÚstm32f1xx_it.cÖÐ
uint32_t HalTime1=0, HalTime2=0;
uint32_t Get_Distance(uint8_t sensor_num) // ´«ÈëÊ±¼äµ¥Î»10us
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
        // 传感器2: PA4=TRIG, PA5=ECHO
        trig_pin = GPIO_PIN_4;
        trig_port = GPIOA;
        echo_pin = GPIO_PIN_5;
        echo_port = GPIOA;
    }

    /* 将Trig引脚拉高10微秒 */
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);
    
    // 如果Echo引脚是低电平则一直等待，直到Echo引脚为高电平，为高电平就说明接收到了返回的信号
    HalTime1 = TimeCounter;
    while(HAL_GPIO_ReadPin(echo_port, echo_pin) == 0)
    {
        if(TimeCounter - HalTime1 > 2200)  break;  //10us * 2000      
    }
    
    // 记录下此时的时间值（10us定时器中累积的变量，所以时间单位为10us）
    HalTime1 = TimeCounter;
    while (HAL_GPIO_ReadPin(echo_port, echo_pin) == 1)  // 等待高电平时间结束
    {
        if(TimeCounter - HalTime1 > 4500)  break;
    }
    
    // 计算时间 判断时间变量是不是从头开始累积了（变量超过范围时会从0开始累积）
    if (TimeCounter > HalTime1)
    {
        // 计算高电平的时间长度
        HalTime2 = TimeCounter - HalTime1;
        Distance = (uint32_t)(((float)HalTime2 * 17) / 100);
    }
    
    if(Distance > 2)
        return Distance;
    else 
        return 1;
}

void Test_Ultrasonic()
{
	uint16_t Dis1 = 0.0;
	uint16_t Dis2 = 0.0;
	char txt[32];
	Ultrasonic_Init();
	OLED_Init();						 // OLED³õÊ¼»¯
	OLED_P6x8Str(10, 0, "Test HCSR04 "); // ×Ö·û´®
	delay_ms(5);
	while (1)
	{
		Dis1 = Get_Distance(1);
		Dis2 = Get_Distance(2);
		sprintf(txt, "Dis=%3d cm", Dis);
		OLED_P8x16Str(10, 2, txt); // ÏÔÊ¾×Ö·û´®
	
		LED_Ctrl(RVS);
		delay_ms(100);
	}
}
