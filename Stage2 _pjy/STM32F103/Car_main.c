#include "Car_main.h"
#include "include.h"

#include "stdlib.h"

#define WAY_Flag 1		//ÈüµÀÑ¡Ôñ£¬1:ºÚµ×°×Ïß	0:°×µ×ºÚÏß

Car Target_V; // Ä¿±ê×ªËÙ
Car ENC_V;	  // Êµ¼Ê×ªËÙ
Car Moto_PWM; // ¿ØÖÆµç»ú¼ÆËã³öµÄPWM

uint8_t Motor_Flag = 0; // µç»úÆôÍ£±êÖ¾Î»		0£ºÍ£Ö¹ÔËÐÐ 1£º¿ªÊ¼ÔË¶¯
float E_V = 0.0;
int32_t car_V;
sensor Car_sensor;
char sensor_info[7];

void Car_main(void)
{
	
	LED_Init();						//³õÊ¼»¯LED	
	KEY_Init();						//³õÊ¼»¯°´¼ü
	OLED_Init();    				//OLED³õÊ¼»¯
	OLED_CLS();						//ÇåÆÁ
	uart_init(USART_2,115200);		//³õÊ¼»¯´®¿Ú
	uart_init(USART_3,115200);		//³õÊ¼»¯´®¿Ú
	Encoder_Init_TIM2();			//±àÂëÆ÷³õÊ¼»¯
	Encoder_Init_TIM3();			//±àÂëÆ÷³õÊ¼»¯
	Encoder_Init_TIM4();			//±àÂëÆ÷³õÊ¼»¯
	MotorInit();					//µç»ú³õÊ¼»¯
	sensor_init();                  //¹âµç´«¸ÐÆ÷³õÊ¼»¯
	
	while(1)
	{
		Motor_Flag = 1;	// ¿ªÊ¼ÔË¶¯
		OLED_Task();		//ÆÁÄ»ÏÔÊ¾		
	}
}
/*********************************************************************
* º¯ÊýÃû £ºvoid car_tim(void)
* ²Î  Êý £ºÎÞ
* ·µ»ØÖµ £ºÎÞ
* ¹¦  ÄÜ £ºÈýÂÖÐ¡³µ Ñ­Ïß³ÌÐò
* Ëµ  Ã÷ £ºÌá¹©Á½ÖÖÈüµÀ£¬°×ÏßºÚµ×¡¢ºÚÏß°×µ×£¬WAY_Flag = 1:ºÚµ×°×Ïß	0:°×µ×ºÚÏß
* µ×É«ÓëÏßÉ«²î´ó¼´¿É£¬¾ßÌå¿ÉÒÔµ÷½Ú¹âµçÁéÃô¶È
********************************************************************/
void car_tim(void)
{
#if	WAY_Flag
	Car_sensor.a = Read_sensor(sensor1);
	Car_sensor.b = Read_sensor(sensor2);
	Car_sensor.c = Read_sensor(sensor3);
	Car_sensor.d = Read_sensor(sensor4);
#else
	Car_sensor.a = -Read_sensor(sensor1);
	Car_sensor.b = -Read_sensor(sensor2);
	Car_sensor.c = -Read_sensor(sensor3);
	Car_sensor.d = -Read_sensor(sensor4);

#endif
	E_V = (Car_sensor.a * 2 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 2);

    Send_Sensor_Data();
}

void OLED_Task(void)
{
	char txt[64];
	
	// ÆÁÄ»ÏÔÊ¾
	sprintf(txt, "%d %d %d %d E:%.1f", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
	OLED_P6x8Str(0, 2, txt); // ×Ö·û´®
	sprintf(txt, "ENC: L:%d R:%d B:%d ", ENC_V.L, ENC_V.R, ENC_V.B);
	OLED_P6x8Str(0, 3, txt); // ×Ö·û´®
	sprintf(txt, "Tar: L:%d R:%d B:%d", Target_V.L, Target_V.R, Target_V.B);
	OLED_P6x8Str(0, 4, txt); // ×Ö·û´®
	printf("samples:%d,%d,%d\n", ENC_V.L, ENC_V.R, ENC_V.B);
	delay_ms(100);
}

char get_info(uint8_t index){
    if(index!=0){
        return '1';
	}
	else{
		return '0';
	}

}

void Send_Sensor_Data(void)
{
#if	WAY_Flag
	sensor_info[0]=get_info(Car_sensor.a);
	sensor_info[1]=get_info(Car_sensor.b);
	sensor_info[2]=get_info(Car_sensor.c);
	sensor_info[3]=get_info(Car_sensor.d);
    sensor_info[4]='+';
	sensor_info[6]='\n';
#else
	sensor_info[0]=get_info(Car_sensor.a);
	sensor_info[1]=get_info(Car_sensor.b);
	sensor_info[2]=get_info(Car_sensor.c);
	sensor_info[3]=get_info(Car_sensor.d);
	sensor_info[4]='-';
	sensor_info[6]='\n';

#endif

    uart_SendBuf(&USART3_Handler, (uint8_t*)sensor_info);
}