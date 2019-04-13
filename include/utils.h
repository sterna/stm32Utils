/*
 *	utils.h
 *
 *	Created on: 31 dec 2014
 *		Author: Sterna
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#ifndef _BV
#define _BV(x) (1<<x)
#endif

typedef enum
{
	COL_RED=1,
	COL_GREEN=2,
	COL_BLUE=3
}colour_t;

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}RGB_t;

char* ftostr(char* buf, float value, unsigned char places);
float xatof(char *s);
signed long xatoi2(char* s);
unsigned char xisdigit(char c);
//Various small math functions
uint32_t utilRandRange(uint32_t range);
uint32_t utilIncWithDir(uint32_t val, int8_t dir, uint32_t rate, uint32_t min, uint32_t max);
int32_t utilLoopValue(int32_t val, int32_t diff, int32_t min, int32_t max);
int32_t utilBounceValue(int32_t val, int32_t diff, int32_t min, int32_t max,int8_t* dir);
inline uint32_t utilIncAndWrapTo0(uint32_t val, uint32_t max);
int8_t utilSign(int32_t v);

//CLock setting functions
void utilSetClockGPIO(GPIO_TypeDef* gpio, FunctionalState state);
void utilSetClockUSART(USART_TypeDef* usart, FunctionalState state);
void utilSetClockSPI(SPI_TypeDef* spi, FunctionalState state);
void utilSetClockDMA(DMA_Channel_TypeDef* dmaCh,FunctionalState state);
void utilSetClockADC(ADC_TypeDef* adc, FunctionalState state);
void utilSetClockTIM(TIM_TypeDef* tim, FunctionalState state);
void utilInitTIMOC(TIM_TypeDef* TIMx, uint8_t ch, TIM_OCInitTypeDef* TIM_OCInitStruct);
void utilTIMWriteOC(TIM_TypeDef* tim, uint8_t ch, uint32_t val);

uint8_t utilGetGPIOPortSource(GPIO_TypeDef* gpio);
#endif /* UTILS_H_ */
