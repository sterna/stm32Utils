/*
 *	utils.c
 *
 *	Created on: 31 dec 2014
 *		Author: Sterna
 *		Contains some useful functions
 */

#include "utils.h"
#include <ctype.h>
#include <math.h>
#include "xprintf.h"


/*
 * Can be used with printf when double values are not
 * supported (because hardfloat is used)
 * Buf is a pointer to a buffer where the output will be written
 * Float is the value to be converted
 * Places is the number of decimals to be used
 * If no buffer is provided, the function will use its own buffer
 */
char* ftostr(char* buf, float value, unsigned char places)
{
	static char _buffer[15];
	unsigned long whole;
//	char sign[2] = "";

//	if (value < 0) {
//		value = -value;
//		sign[0] = '-';
//		sign[1] = '\0';
//	}

	whole = (signed long) value;
	char fracBuffer[10];
	fracBuffer[places]='\0';
	for (unsigned char i=0;i<places;i++)
	{
		value=value*10.0;
		fracBuffer[i]=(unsigned char)('0'+(signed long)(value)%10);
	}
	if (buf)
	{
		xsprintf(buf, "%D.%s",whole,fracBuffer);
		return 0;
	}
	else
	{
		xsprintf(_buffer, "%D.%s",whole,fracBuffer);
		return _buffer;
	}
	//xsprintf(_buffer, "%s%lu.%*.*lu", sign, whole, places, places, fraction); //xsprintf does not support *
}

/*
 * Convert a string into a float.
 * Will return 0 if any incorrect character is received
 * Does not support scientific notification (E)
 */
float xatof(char *s)
{
	float a = 0.0;
	unsigned short magnitude = 1;
	signed char sign=1;
	unsigned long whole=0;
	unsigned long frac=0;
	unsigned char analysingFrac=0;
	unsigned char c;
	if(*s == '-')
	{
		sign=-1;
		s++;
	}
	while ((c = *s++) != '\0')
	{
		if(xisdigit(c))
		{
			if(!analysingFrac)
			{
				whole = whole*10 + (c - '0');
			}
			else
			{
				frac = frac*10 + (c - '0');
				magnitude=magnitude*10;
			}
		}
		else if(c=='.')
		{
			analysingFrac=1;
		}
		else
		{
			//Error, unknown character, return "safe" value
			return 0;
		}
	}
	a=(float)(whole)+(float)frac/(float)magnitude;
	a=a*((float)(sign));

	return a;
}

/*
 *	Convert a string into an integer
 *	The string needs to nullterminated
 */
signed long xatoi2(char* s)
{
	signed char sign=1;
	unsigned char c=0;
	signed long out=0;
	//Skip leading spaces (if any)
	if(*s==' ')
	{
		while(*++s==' '){}
	}
	//Is negative?
	if(*s == '-')
	{
		sign=-1;
		s++;
	}
	while((c=*s++)!='\0')
	{
		if(xisdigit(c))
		{
			out=out*10 + (c-'0');
		}
		else
		{
			//Error
			return 0;
		}
	}
	return out*sign;
}

unsigned char xisdigit(char c)
{
	return ((c>='0') && (c<='9'));
}

/*
 * Returns a random value between 0 and range
 */
uint32_t utilRandRange(uint32_t range)
{
	uint32_t val=rand();
	range++; //otherwise, it will never return range, since it could only happen at rand=RAND_MAX
	return (uint32_t)((val-1)/(RAND_MAX/range));
}

/*
 * Increase or decrease values based on certain parameters
 */
uint32_t utilIncWithDir(uint32_t val, int8_t dir, uint32_t rate, uint32_t min, uint32_t max)
{
	if(val<rate && dir==-1)
	{
		val=0;
	}
	else
	{
		val += dir*rate;
		if(val>max)
		{
			val=max;
		}
		else if (val<min)
		{
			val=min;
		}
	}
	return val;
}

/*
 * Will perform a wraparound of the value, loop style (if going over max, it restarts at min or vice versa)
 * min and max are allowed values
 * The distance between min and max are considered to be one (hence max+1->min and min-1->max)
 * If values are outside of bounds for int32, something weird might happen
 * Weird stuff might happen if min>max
 */
int32_t utilLoopValue(int32_t val, int32_t diff, int32_t min, int32_t max)
{
	uint32_t range=max-min;
	val=val+diff;
	while(val<min)
	{
		val+=range+1;
	}
	while(val>max+1)
	{
		val-=range;
	}
	return val;
}

/*
 * Will perform a bounce off an edge value (if going past min or max, it will go in the reverse direction back)
 * min and max are allowed values
 * if given, the function will put the resulting direction into dir, for the caller to get (C can only return one value)
 * If values are outside of bounds for int32, something weird might happen
 * Weird stuff might happen if min>max
 */
int32_t utilBounceValue(int32_t val, int32_t diff, int32_t min, int32_t max,int8_t* dir)
{
	int8_t dirAfter=utilSign(diff);
	val+=diff;
	while(val>max || val<min)
	{
		if(val>max)
		{
			val=2*max-val;
			dirAfter=-1;
		}
		if(val<min)
		{
			val=2*min-val;
			dirAfter=1;
		}
	}
	if(dir!=NULL)
	{
		*dir=dirAfter;
	}
	return val;
}

/*
 * increments a value and wraps it to 0 if too large
 */
inline uint32_t utilIncAndWrapTo0(uint32_t val, uint32_t max)
{
	val++;
	if(val>max)
	{
		val=0;
	}
	return val;

}

/*
 * Returns the sign of the value
 */
int8_t utilSign(int32_t v)
{
	if(v>=0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}

/*
 * Sets the clock for a given GPIO
 */
void utilSetClockGPIO(GPIO_TypeDef* gpio, FunctionalState state)
{
#ifdef RCC_APB1Periph_GPIOA
	if(gpio==GPIOA)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOA,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOA
	if(gpio==GPIOA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOA
	if(gpio==GPIOA)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOA,state);
	}
#endif

#ifdef RCC_APB1Periph_GPIOB
	if(gpio==GPIOB)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOB,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOB
	if(gpio==GPIOB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOB
	if(gpio==GPIOB)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOB,state);
	}
#endif

#ifdef RCC_APB1Periph_GPIOC
	if(gpio==GPIOC)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOC,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOC
	if(gpio==GPIOC)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOC
	if(gpio==GPIOC)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOC,state);
	}
#endif

#ifdef RCC_APB1Periph_GPIOD
	if(gpio==GPIOD)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOD,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOD
	if(gpio==GPIOD)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOD
	if(gpio==GPIOD)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOD,state);
	}
#endif

#ifdef RCC_APB1Periph_GPIOE
	if(gpio==GPIOE)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOE,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOE
	if(gpio==GPIOE)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOE
	if(gpio==GPIOE)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOE,state);
	}
#endif

#ifdef RCC_APB1Periph_GPIOF
	if(gpio==GPIOF)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_GPIOF,state);
	}
#endif
#ifdef RCC_APB2Periph_GPIOF
	if(gpio==GPIOF)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,state);
	}
#endif
#ifdef RCC_APB3Periph_GPIOF
	if(gpio==GPIOF)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_GPIOF,state);
	}
#endif
}


/*
 * Sets the clock for a given SPI
 */
void utilSetClockSPI(SPI_TypeDef* spi, FunctionalState state)
{
#ifdef RCC_APB1Periph_SPI1
	if(spi==SPI1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI1,state);
	}
#endif
#ifdef RCC_APB2Periph_SPI1
	if(spi==SPI1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,state);
	}
#endif
#ifdef RCC_APB3Periph_SPI1
	if(spi==SPI1)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_SPI1,state);
	}
#endif

#ifdef RCC_APB1Periph_SPI2
	if(spi==SPI2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,state);
	}
#endif
#ifdef RCC_APB2Periph_SPI2
	if(spi==SPI2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI2,state);
	}
#endif
#ifdef RCC_APB3Periph_SPI2
	if(spi==SPI2)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_SPI2,state);
	}
#endif

#ifdef RCC_APB1Periph_SPI3
	if(spi==SPI3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,state);
	}
#endif
#ifdef RCC_APB2Periph_SPI3
	if(spi==SPI3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI3,state);
	}
#endif
#ifdef RCC_APB3Periph_SPI3
	if(spi==SPI3)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_SPI3,state);
	}
#endif

#ifdef RCC_APB1Periph_SPI4
	if(spi==SPI4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI4,state);
	}
#endif
#ifdef RCC_APB2Periph_SPI4
	if(spi==SPI4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4,state);
	}
#endif
#ifdef RCC_APB3Periph_SPI4
	if(spi==SPI4)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_SPI4,state);
	}
#endif

}


/*
 * Sets the clock for a given USART
 */
void utilSetClockUSART(USART_TypeDef* usart, FunctionalState state)
{
#ifdef RCC_APB1Periph_USART1
	if(usart==USART1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART1,state);
	}
#endif
#ifdef RCC_APB2Periph_USART1
	if(usart==USART1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,state);
	}
#endif
#ifdef RCC_APB3Periph_USART1
	if(usart==USART1)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_USART1,state);
	}
#endif

#ifdef RCC_APB1Periph_USART2
	if(usart==USART2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,state);
	}
#endif
#ifdef RCC_APB2Periph_USART2
	if(usart==USART2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART2,state);
	}
#endif
#ifdef RCC_APB3Periph_USART2
	if(usart==USART2)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_USART2,state);
	}
#endif

#ifdef RCC_APB1Periph_USART3
	if(usart==USART3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,state);
	}
#endif
#ifdef RCC_APB2Periph_USART3
	if(usart==USART3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART3,state);
	}
#endif
#ifdef RCC_APB3Periph_USART3
	if(usart==USART3)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_USART3,state);
	}
#endif

#ifdef RCC_APB1Periph_USART4
	if(usart==USART4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4,state);
	}
#endif
#ifdef RCC_APB2Periph_USART4
	if(usart==USART4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART4,state);
	}
#endif
#ifdef RCC_APB3Periph_USART4
	if(usart==USART4)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_USART4,state);
	}
#endif

#ifdef RCC_APB1Periph_UART4
	if(usart==UART4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,state);
	}
#endif
#ifdef RCC_APB2Periph_UART4
	if(usart==UART4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART4,state);
	}
#endif
#ifdef RCC_APB3Periph_UART4
	if(usart==UART4)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_UART4,state);
	}
#endif


#ifdef RCC_APB1Periph_USART5
	if(usart==USART5)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART5,state);
	}
#endif
#ifdef RCC_APB2Periph_USART5
	if(usart==USART5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART5,state);
	}
#endif
#ifdef RCC_APB3Periph_USART5
	if(usart==USART5)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_USART5,state);
	}
#endif

#ifdef RCC_APB1Periph_UART5
	if(usart==UART5)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,state);
	}
#endif
#ifdef RCC_APB2Periph_UART5
	if(usart==UART5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART5,state);
	}
#endif
#ifdef RCC_APB3Periph_UART5
	if(usart==UART5)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_UART5,state);
	}
#endif
}

/*
 * Sets a DMA clock based on a DMA channel
 */
void utilSetClockDMA(DMA_Channel_TypeDef* dmaCh,FunctionalState state)
{
#ifdef RCC_AHBPeriph_DMA1
	if(dmaCh == DMA1_Channel1 || dmaCh == DMA1_Channel2 || dmaCh == DMA1_Channel3 || dmaCh == DMA1_Channel4 || dmaCh == DMA1_Channel5 || dmaCh == DMA1_Channel6 || dmaCh == DMA1_Channel7)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,state);
	}
#endif
#ifdef RCC_AHBPeriph_DMA2
	if(dmaCh == DMA2_Channel1 || dmaCh == DMA2_Channel2 || dmaCh == DMA2_Channel3 || dmaCh == DMA2_Channel4 || dmaCh == DMA2_Channel5)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,state);
	}
#endif
}

/*
 * Sets the clock for a given SPI
 */
void utilSetClockADC(ADC_TypeDef* adc, FunctionalState state)
{
#ifdef RCC_APB1Periph_ADC1
	if(adc==ADC1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC1,state);
	}
#endif
#ifdef RCC_APB2Periph_ADC1
	if(adc==ADC1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,state);
	}
#endif
#ifdef RCC_APB3Periph_ADC1
	if(adc==ADC1)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_ADC1,state);
	}
#endif

#ifdef RCC_APB1Periph_ADC2
	if(adc==ADC2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC2,state);
	}
#endif
#ifdef RCC_APB2Periph_ADC2
	if(adc==ADC2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,state);
	}
#endif
#ifdef RCC_APB3Periph_ADC2
	if(adc==ADC2)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_ADC2,state);
	}
#endif

#ifdef RCC_APB1Periph_ADC3
	if(adc==ADC3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_ADC3,state);
	}
#endif
#ifdef RCC_APB2Periph_ADC3
	if(adc==ADC3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,state);
	}
#endif
#ifdef RCC_APB3Periph_ADC3
	if(adc==ADC3)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_ADC3,state);
	}
#endif
}

/*
 * Sets the clock for a given timer
 */
void utilSetClockTIM(TIM_TypeDef* tim, FunctionalState state)
{
#ifdef RCC_APB1Periph_TIM1
	if(tim==TIM1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM1,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM1
	if(tim==TIM1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM1
	if(tim==TIM1)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM1,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM2
	if(tim==TIM2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM2
	if(tim==TIM2)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM2,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM2
	if(tim==TIM2)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM2,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM3
	if(tim==TIM3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM3
	if(tim==TIM3)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM3,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM3
	if(tim==TIM3)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM3,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM4
	if(tim==TIM4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM4
	if(tim==TIM4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM4,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM4
	if(tim==TIM4)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM4,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM5
	if(tim==TIM5)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM5
	if(tim==TIM5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM5,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM5
	if(tim==TIM5)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM5,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM6
	if(tim==TIM6)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM6
	if(tim==TIM6)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM6,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM6
	if(tim==TIM6)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM6,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM7
	if(tim==TIM7)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM7
	if(tim==TIM7)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM7,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM7
	if(tim==TIM7)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM7,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM8
	if(tim==TIM8)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM8,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM8
	if(tim==TIM8)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM8
	if(tim==TIM8)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM8,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM9
	if(tim==TIM9)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM9,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM9
	if(tim==TIM9)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM9
	if(tim==TIM9)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM9,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM10
	if(tim==TIM10)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM10,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM10
	if(tim==TIM10)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM10
	if(tim==TIM10)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM10,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM11
	if(tim==TIM11)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM11,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM11
	if(tim==TIM11)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM11
	if(tim==TIM11)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM11,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM12
	if(tim==TIM12)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM12
	if(tim==TIM12)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM12,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM12
	if(tim==TIM12)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM12,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM13
	if(tim==TIM13)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM13
	if(tim==TIM13)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM13,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM13
	if(tim==TIM13)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM13,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM14
	if(tim==TIM14)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM14
	if(tim==TIM14)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM14,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM14
	if(tim==TIM14)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM14,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM15
	if(tim==TIM15)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM15,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM15
	if(tim==TIM15)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM15
	if(tim==TIM15)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM15,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM16
	if(tim==TIM16)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM16,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM16
	if(tim==TIM16)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM16
	if(tim==TIM16)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM16,state);
	}
#endif
//NEXT
#ifdef RCC_APB1Periph_TIM17
	if(tim==TIM17)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM17,state);
	}
#endif
#ifdef RCC_APB2Periph_TIM17
	if(tim==TIM17)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,state);
	}
#endif
#ifdef RCC_APB3Periph_TIM17
	if(tim==TIM17)
	{
		RCC_APB3PeriphClockCmd(RCC_APB3Periph_TIM17,state);
	}
#endif
//NEXT
}

/*
 * Inits a certain OC for a timer, based on a number
 */
void utilInitTIMOC(TIM_TypeDef* TIMx, uint8_t ch, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
	switch(ch)
	{
	case 1:
		TIM_OC1Init(TIMx,TIM_OCInitStruct);
		break;
	case 2:
		TIM_OC2Init(TIMx,TIM_OCInitStruct);
		break;
	case 3:
		TIM_OC3Init(TIMx,TIM_OCInitStruct);
		break;
	case 4:
		TIM_OC4Init(TIMx,TIM_OCInitStruct);
		break;
	}
}

/*
 * Write a value to a timer channel, based on a number
 */
void utilTIMWriteOC(TIM_TypeDef* tim, uint8_t ch, uint32_t val)
{
	switch(ch)
	{
	case 1:
		tim->CCR1 = val;
		break;
	case 2:
		tim->CCR2 = val;
		break;
	case 3:
		tim->CCR3 = val;
		break;
	case 4:
		tim->CCR4 = val;
		break;
	}
}

/*
 * Returns the portsource (used for EXTI) based on the port pointer
 */
uint8_t utilGetGPIOPortSource(GPIO_TypeDef* gpio)
{
#ifdef GPIOA
	if(gpio==GPIOA)
	{
		return GPIO_PortSourceGPIOA;
	}
#endif
#ifdef GPIOB
	if(gpio==GPIOB)
	{
		return GPIO_PortSourceGPIOB;
	}
#endif
#ifdef GPIOC
	if(gpio==GPIOC)
	{
		return GPIO_PortSourceGPIOC;
	}
#endif
#ifdef GPIOD
	if(gpio==GPIOD)
	{
		return GPIO_PortSourceGPIOD;
	}
#endif
#ifdef GPIOE
	if(gpio==GPIOE)
	{
		return GPIO_PortSourceGPIOE;
	}
#endif
#ifdef GPIOF
	if(gpio==GPIOF)
	{
		return GPIO_PortSourceGPIOF;
	}
#endif
#ifdef GPIOG
	if(gpio==GPIOG)
	{
		return GPIO_PortSourceGPIOG;
	}
#endif
#ifdef GPIOH
	if(gpio==GPIOH)
	{
		return GPIO_PortSourceGPIOH;
	}
#endif
#ifdef GPIOI
	if(gpio==GPIOI)
	{
		return GPIO_PortSourceGPIOI;
	}
#endif
	return 0;
}
