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
//	LQ_TIM2_Init(71,9);                    // ¶¨Ê±Æ÷ÓÃÓÚ¼ÆÊý
}
extern uint32_t TimeCounter;	 //ÓÃÓÚ³¬Éù²¨¼ÆÊ±,¸Ã±äÁ¿Î»ÓÚstm32f1xx_it.cÖÐ
uint32_t HalTime1=0, HalTime2=0;
uint32_t Get_Distance(void) // ´«ÈëÊ±¼äµ¥Î»10us
{
	uint32_t Distance = 0;
	/*½«TrigÒý½ÅÀ­¸ßÊ®Î¢Ãë*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	//Èç¹ûEchoÒý½ÅÊÇµÍµçÆ½ÔòÒ»Ö±µÈ´ý£¬Ö±µ½ÎªEchoÒý½ÅÎª¸ßµçÆ½£¬Îª¸ßµçÆ½¾ÍËµÃ÷½ÓÊÜµ½ÁË·µ»ØµÄÐÅºÅ
    HalTime1= TimeCounter;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
    {
        if(TimeCounter - HalTime1 > 2200)  break;  //10us *2000      
    }
	//¼ÇÂ¼ÏÂ´ËÊ±µÄÊ±¼äÖµ£¨10us¶¨Ê±Æ÷ÖÐÀÛ¼ÓµÄ±äÁ¿£¬ËùÒÔÊ±¼äµ¥Î»Îª10us£©
//    printf("TIMEA:%d \r\n", TimeCounter - HalTime1); // ´òÓ¡Õý³£¶ÁÈ¡Êý¾ÝÊ±µÄÖµ´óÐ¡
	HalTime1 = TimeCounter;
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)	//µÈ´ý¸ßµçÆ½Ê±¼ä½áÊø
    {
        if(TimeCounter - HalTime1 > 4500)  break;
    }
//    printf("TIMEB:%d \r\n", TimeCounter - HalTime1); // ´òÓ¡Õý³£¶ÁÈ¡Êý¾ÝÊ±µÄÖµ´óÐ¡
	//¼ÆËãÊ±¼ä ÅÐ¶ÏÊ±¼ä±äÁ¿ÊÇ²»ÊÇ´ÓÍ·¿ªÊ¼ÀÛ¼ÓÁË£¨±äÁ¿³¬¹ý·¶Î§Ê±»á´Ó0¿ªÊ¼ÀÛ¼Ó£©
	if (TimeCounter > HalTime1)
	{
		//¼ÆËã¸ßµçÆ½µÄÊ±¼ä³¤¶È
		HalTime2 = TimeCounter - HalTime1;
//        Distance=(uint32_t)((float)HalTime2/58*10);//¾àÀëµ¥Î»cm,ÉùËÙ340M/S£¬Ê±¼ä*ËÙ¶È/2=¾àÀë
        Distance = (uint32_t)(((float)HalTime2 *17)/100);

	}
	if( 2 < Distance)
        return Distance;
    else 
        return 1;
}

void Test_Ultrasonic()
{
	uint16_t Dis = 0.0;
	char txt[32];
	Ultrasonic_Init();
	OLED_Init();						 // OLED³õÊ¼»¯
	OLED_P6x8Str(10, 0, "Test HCSR04 "); // ×Ö·û´®
	delay_ms(5);
	while (1)
	{
		Dis = Get_Distance();
		sprintf(txt, "Dis=%3d cm", Dis);
		OLED_P8x16Str(10, 2, txt); // ÏÔÊ¾×Ö·û´®
	
		LED_Ctrl(RVS);
		delay_ms(100);
	}
}
