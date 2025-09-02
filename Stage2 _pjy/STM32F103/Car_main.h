#ifndef _CAR_MAIN_H
#define _CAR_MAIN_H
#include "stdint.h"

typedef struct
{
    int32_t L;
    int32_t R;
    int32_t B;
} Car;

typedef enum
{
    idle = 0, // 0:¿ÕÏÐ-Ô­µØÖ±Á¢
    run,      // 1:×ÔÓÉÅÜ
    barrier,  // 2:¿´µ½ÕÏ°­Îï
    find_R_way, // 3:Õý³£Ñ­¼£
    find_delay, // 4:Õý³£Ñ­¼£
    find_L_way, // 5:Õý³£Ñ­¼£
    find_delay_V, // 6:Õý³£Ñ­¼£
    find_way,   //7
    leisure   // ÔÝÊ±²»ÓÃ
} State_car;

typedef struct
{
    int8_t a;
    int8_t b;
    int8_t c;
    int8_t d;
} sensor;


typedef struct {
	char flag;     //±êÖ¾Î»
	int  L_V;  //×óÂÖËÙ¶È
	int  R_V;  //ÓÒÂÖËÙ¶È
	int  B_V;  //ºóÂÖËÙ¶È
	
} Color_V;

void Car_main(void);
void car_tim(void);
void SYS_Task(void);
void OLED_Task(void);

void Send_Sensor_Data(void);

#endif
