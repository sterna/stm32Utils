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

#ifndef PI
#define PI	3.14159f
#endif

//The max value the ADC can use (12bit)
#define ADC_MAX_VAL	4096
//Used to convert an ADC-value to millivolt
#define MILLIVOLT_PER_10BIT		(33000000/ADC_MAX_VAL)

//Get the GPIO_pin (used for GPIO init etc) from a pin number
#define utilGetGPIOPin(pin) 	(uint16_t)(1<<pin)
//Get the ExtiLine value from a pin number
#define utilGetExtiLine(pin)	(uint32_t)(1<<pin)
//Get the Timer channel interrupt flag from a channel number
#define utilGetTimCC_IT(x)		(uint16_t)(1<<x)

#define utilReadU16_lsb(byte_vector, byte_offset) (byte_vector[byte_offset] | (byte_vector[byte_offset + 1] << 8))

#define utilConvertADCtoMillivolt(x) ((MILLIVOLT_PER_10BIT*((uint32_t)x))/10000)

//Returns the max between x and y. Probably not 100% typesafe
#define utilMax(x,y)	(x>y ? x : y)

#define UTIL_GET_IRQn(x)		x##_IRQn
#define UTIL_GET_IRQHandler(x)	x##_IRQHandler

//Converts mm^3 to nanoliters
#define UTIL_MM3_TO_NL(x)	(x*1000.0)
#define UTIL_NL_TO_MM3(x)	(x/1000.0)

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
void utilRandSeed(uint32_t seed);
uint32_t utilIncWithDir(uint32_t val, int8_t dir, uint32_t rate, uint32_t min, uint32_t max);
int32_t utilLoopValue(int32_t val, int32_t diff, int32_t min, int32_t max);
int32_t utilBounceValue(int32_t val, int32_t diff, int32_t min, int32_t max,int8_t* dir);
int8_t utilSign(int32_t v);
//Line functions
int32_t utilLineGetXPoint(int32_t x1, int32_t y1, int32_t y, int32_t k);
float utilLineGetXPointF(float x1, float y1, float y, float k);
int32_t utilLineGetYPoint(int32_t x1, int32_t y1, int32_t x, int32_t k);
float utilLineGetYPointF(float x1, float y1, float x, float k);
int32_t utilLineFindSlope(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
float utilLineFindSlopeF(float x1, float y1, float x2, float y2);
int32_t utilLineFindSlopeRel(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
float utilLineFindSlopeRelF(float x1, float y1, float x2, float y2);
int32_t utilLineInterpolate(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2);
float utilLineInterpolateF(float x, float x1, float y1, float x2, float y2);

uint32_t utilSqrtI2I(uint32_t v);

//Explicit data conversion functions
void utilUint32To4Bytes(uint32_t in, uint8_t* out,bool lsbFirst);
void utilUint16To2Bytes(uint16_t in, uint8_t* out,bool lsbFirst);
void utilInt16To2Bytes(int16_t in, uint8_t* out,bool lsbFirst);
void utilInt32To4Bytes(int32_t in, uint8_t* out,bool lsbFirst);
uint32_t util4BytesToUint32(uint8_t* data, bool lsbFirst);
uint16_t util2BytesToUint16(uint8_t* data, bool lsbFirst);
int16_t util2BytesToInt16(uint8_t* data, bool lsbFirst);
int32_t util4BytesToInt32(uint8_t* data, bool lsbFirst);
uint8_t utilGetBitNumber(uint32_t val);

//Clock setting functions
void utilSetClockGPIO(GPIO_TypeDef* gpio, FunctionalState state);
void utilSetClockUSART(USART_TypeDef* usart, FunctionalState state);
void utilSetClockSPI(SPI_TypeDef* spi, FunctionalState state);
void utilSetClockDMA(DMA_Channel_TypeDef* dmaCh,FunctionalState state);
void utilSetClockADC(ADC_TypeDef* adc, FunctionalState state);
void utilSetClockTIM(TIM_TypeDef* tim, FunctionalState state);
void utilSetClockAFIO(FunctionalState state);

//Functions for Timer OC
void utilTIMOCInit(TIM_TypeDef* TIMx, uint8_t ch, TIM_OCInitTypeDef* TIM_OCInitStruct);
void utilTIMOCConfigPreload(TIM_TypeDef* TIMx, uint8_t ch, uint16_t timOcPreload);
void utilTIMOCWrite(TIM_TypeDef* tim, uint8_t ch, uint32_t val);

uint8_t utilGetGPIOPortSource(GPIO_TypeDef* gpio);

#endif /* UTILS_H_ */
