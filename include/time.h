#ifndef __TIME_H__
#define __TIME_H__

#include "stm32f10x.h"

typedef struct{
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
}timeStruct;


//-----------------------------------------------------------------------
// External functions
//-----------------------------------------------------------------------
//
void timeInit(void);
void delay_ms(unsigned long ms);
void delay_us(unsigned long us);
unsigned long microSeconds();
void timeFormat(timeStruct* outStruct,unsigned long seconds);

void SysTick_Handler(void);
//-----------------------------------------------------------------------

extern volatile uint32_t systemTime;

#endif


