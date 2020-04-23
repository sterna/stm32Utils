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
 * Sends a seed to the rand function
 */
void utilRandSeed(uint32_t seed)
{
	srand(seed);
}

/*
 * Increase or decrease values based on certain parameters
 * Min and max values are included
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
 * Increments a value until (including) max. When max is reached, it goes back to 0
 */
uint32_t utilIncLoopSimple(uint32_t val, uint32_t max)
{
	val++;
	if(val>max)
	{
		val=0;
	}
	return val;
}

/*
 * Decrements a down to and including 0. When 0 is reached, it jumps to max
 */
uint32_t utilDecLoopSimple(uint32_t val, uint32_t max)
{
	if(val)
	{
		val--;
	}
	else
	{
		val=max;
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
 * Return true if a value is close enough to a given threshold with a given tolerance
 * The edge values are included
 */
bool utilValCloseEnough(int32_t val, int32_t thresh, int32_t tol)
{
	if((val>=(thresh-tol)) && (val<=(thresh+tol)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Return true if a value is close enough to a given threshold with a given tolerance
 * The edge values are included
 */
bool utilValCloseEnoughDual(int32_t val1, int32_t val2, int32_t thresh1, int32_t thresh2, int32_t tol)
{
	if(utilValCloseEnough(val1,thresh1,tol) && utilValCloseEnough(val2,thresh2,tol))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Scales a value from one range to another (same as Arduino map)
 * Note that inMax!=inMax
 */
int32_t utilMap(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax)
{
	return ((in - inMin) * (outMax - outMin)) / (inMax - inMin) + outMin;
}

/*
 * Simplified version of utilMap, but assumes the min values are 0
 * Note that scaleMax and scale must be positive integers and that scaleMax must be >0
 */
int32_t utilScale(int32_t in, uint32_t scaleMax, uint32_t scale)
{
	return (in*scale)/scaleMax;
}

/*
 * Converts two bytes into an uint16_t. Useful when getting data from the printheads
 * data is a pointer to the data where the uint16 starts. Will always assume 2 bytes of data
 * Note: This is very non-atomic
 */
uint16_t util2BytesToUint16(uint8_t* data, bool lsbFirst)
{
	uint16_t tmp=0;
	if(lsbFirst)
	{
		tmp=(uint16_t)data[0];
		tmp+=(uint16_t)(data[1]<<8);
	}
	else
	{
		tmp=(uint16_t)data[1];
		tmp+=(uint16_t)(data[0]<<8);
	}
	return tmp;
}

/*
 * Converts two bytes into an int16_t. Useful when getting data from the printheads
 * data is a pointer to the data where the int16 starts. Will always assume 2 bytes of data
 * Note: This is very non-atomic
 */
int16_t util2BytesToInt16(uint8_t* data, bool lsbFirst)
{
	uint16_t tmp=0;
	if(lsbFirst)
	{
		tmp=(uint16_t)data[0];
		tmp+=(uint16_t)(data[1]<<8);
	}
	else
	{
		tmp=(uint16_t)data[1];
		tmp+=(uint16_t)(data[0]<<8);
	}
	return (int16_t)(tmp-INT16_MAX);
}

/*
 * Converts four bytes into a uint32_t. Useful when getting data from the printheads
 * data is a pointer to the data where the uint32 starts. Always assumes 4 bytes
 * Note: This is very non-atomic
 */
uint32_t util4BytesToUint32(uint8_t* data, bool lsbFirst)
{
	uint32_t tmp=0;
	if(lsbFirst)
	{
		tmp=(uint32_t)data[0];
		tmp+=(uint32_t)(data[1]<<8);
		tmp+=(uint32_t)(data[2]<<16);
		tmp+=(uint32_t)(data[3]<<24);
	}
	else
	{
		tmp=(uint32_t)data[3];
		tmp+=(uint32_t)(data[2]<<8);
		tmp+=(uint32_t)(data[1]<<16);
		tmp+=(uint32_t)(data[0]<<24);
	}
	return tmp;
}

/*
 * Converts four bytes into a int32_t. Useful when getting data from the printheads
 * data is a pointer to the data where the int32 starts. Always assumes 4 bytes
 * Note: This is very non-atomic
 */
int32_t util4BytesToInt32(uint8_t* data, bool lsbFirst)
{
	uint32_t tmp=0;
	if(lsbFirst)
	{
		tmp=(uint32_t)data[0];
		tmp+=(uint32_t)(data[1]<<8);
		tmp+=(uint32_t)(data[2]<<16);
		tmp+=(uint32_t)(data[3]<<24);
	}
	else
	{
		tmp=(uint32_t)data[3];
		tmp+=(uint32_t)(data[2]<<8);
		tmp+=(uint32_t)(data[1]<<16);
		tmp+=(uint32_t)(data[0]<<24);
	}
	return (int32_t)(tmp-INT32_MAX);
}

/*
 * Takes a uint16_t and inserts it into an array with two bytes
 * Note: This is very non-atomic
 */
void utilUint16To2Bytes(uint16_t in, uint8_t* out,bool lsbFirst)
{
	if(lsbFirst)
	{
		out[0]=(uint16_t)(in&0xFF);
		out[1]=(uint16_t)((in>>8)&0xFF);
	}
	else
	{
		out[1]=(uint16_t)(in&0xFF);
		out[0]=(uint16_t)((in>>8)&0xFF);
	}
}

/*
 * Takes an int16_t and inserts it into an array with two bytes
 * Note: This is very non-atomic
 */
void utilInt16To2Bytes(int16_t in, uint8_t* out,bool lsbFirst)
{
	uint16_t tmp=(uint16_t)(in+INT16_MAX);
	if(lsbFirst)
	{
		out[0]=(uint16_t)(tmp&0xFF);
		out[1]=(uint16_t)((tmp>>8)&0xFF);
	}
	else
	{
		out[1]=(uint16_t)(tmp&0xFF);
		out[0]=(uint16_t)((tmp>>8)&0xFF);
	}
}

/*
 * Takes a uint32_t and inserts it into an array with four bytes
 * Note: This is very non-atomic
 */
void utilUint32To4Bytes(uint32_t in, uint8_t* out,bool lsbFirst)
{
	if(lsbFirst)
	{
		out[0]=(uint16_t)(in&0xFF);
		out[1]=(uint16_t)((in>>8)&0xFF);
		out[2]=(uint16_t)((in>>16)&0xFF);
		out[3]=(uint16_t)((in>>24)&0xFF);
	}
	else
	{
		out[3]=(uint16_t)(in&0xFF);
		out[2]=(uint16_t)((in>>8)&0xFF);
		out[1]=(uint16_t)((in>>16)&0xFF);
		out[0]=(uint16_t)((in>>24)&0xFF);
	}
}

/*
 * Takes an int32_t and inserts it into an array with four bytes
 * Note: This is very non-atomic
 */
void utilInt32To4Bytes(int32_t in, uint8_t* out,bool lsbFirst)
{
	uint32_t tmp=(uint32_t)(in+INT32_MAX);
	if(lsbFirst)
	{
		out[0]=(uint16_t)(tmp&0xFF);
		out[1]=(uint16_t)((tmp>>8)&0xFF);
		out[2]=(uint16_t)((tmp>>16)&0xFF);
		out[3]=(uint16_t)((tmp>>24)&0xFF);
	}
	else
	{
		out[3]=(uint16_t)(tmp&0xFF);
		out[2]=(uint16_t)((tmp>>8)&0xFF);
		out[1]=(uint16_t)((tmp>>16)&0xFF);
		out[0]=(uint16_t)((tmp>>24)&0xFF);
	}
}

/*
 * Will give you the bit number of the highest bit set for a value
 * A value of 1 will return 0 (the first bit is bit 0)
 * If no bit is set, it will return UINT8_MAX
 */
uint8_t utilGetBitNumber(uint32_t val)
{
	if(!val)
	{
		return UINT8_MAX;
	}
	uint8_t cnt=0;
	while(val)
	{
		val=val>>1;
		cnt++;
	}
	return cnt-1;
}

/*
 * Returns an X-point on a straight line, given a point (x1,y1), a slope (k), and y-coordinate for the other point
 */
int32_t utilLineGetXPoint(int32_t x1, int32_t y1, int32_t y, int32_t k)
{
	return (x1-(y1-y)/k);
}

/*
 * Returns an X-point on a straight line, given a point (x1,y1), a slope (k), and y-coordinate for the other point
 * Supports floating point
 */
float utilLineGetXPointF(float x1, float y1, float y, float k)
{
	return (x1-(y1-y)/k);
}

/*
 * Returns an Y-point on a straight line, given a point (x1,y1), a slope (k), and x-coordinate for the other point
 */
int32_t utilLineGetYPoint(int32_t x1, int32_t y1, int32_t x, int32_t k)
{
	return (y1-k*(x1-x));
}

/*
 * Returns an Y-point on a straight line, given a point (x1,y1), a slope (k), and x-coordinate for the other point
 * Supports floating point
 */
float utilLineGetYPointF(float x1, float y1, float x, float k)
{
	return (y1-k*(x1-x));
}

/*
 * Finds the slope (k) for a line given 2 points {{x1,y1},{x2,y2}}
 * This satisfies the function y=kx+m
 */
int32_t utilLineFindSlope(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	return (y1-y2)/(x1-x2);
}

/*
 * Finds the slope (k) for a line given 2 points {{x1,y1},{x2,y2}}
 * This satisfies the function y=kx+m
 * Floating point version
 */
float utilLineFindSlopeF(float x1, float y1, float x2, float y2)
{
	return (y1-y2)/(x1-x2);
}

/*
 * Finds the slope (k) for a line given 2 points {{x1,y1},{x2,y2}},
 * relative to Y2
 */
int32_t utilLineFindSlopeRel(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	return (y2/y1-1)/(x1-x2);
}

/*
 * Finds the slope (k) for a line given 2 points {{x1,y1},{x2,y2}},
 * relative to Y2
 * Floating point version
 */
float utilLineFindSlopeRelF(float x1, float y1, float x2, float y2)
{
	return (y2/y1-1)/(x1-x2);
}

/*
 * Linear interpolate a value between two know points
 */
int32_t utilLineInterpolate(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	const int32_t k=utilLineFindSlope(x1,y1,x2,y2);
	return k*x+y1-k*x1;
}

/*
 * Linear interpolate a value between two know points
 * Floating point version
 */
float utilLineInterpolateF(float x, float x1, float y1, float x2, float y2)
{
	const float k=utilLineFindSlopeF(x1,y1,x2,y2);
	return k*x+y1-k*x1;
}


/*
	MIT License

	Copyright (c) 2019 Christophe Meessen

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

	License for the integer square root
 */

/*
 * uint32_t sqrtI2I(uint32_t v);
 *
 * Compute uint32_t to uint32_t square root
 * RETURNS the integer square root of v
 * REQUIRES v is positive (or sadness will fall :( )
 */
uint32_t utilSqrtI2I(uint32_t v)
{
    uint32_t t, q, b, r;
    r = v;           // r = v - x²
    b = 0x40000000;  // a²
    q = 0;           // 2ax
    while( b > 0 )
    {
        t = q + b;   // t = 2ax + a²
        q >>= 1;     // if a' = a/2, then q' = q/2
        if( r >= t ) // if (v - x²) >= 2ax + a²
        {
            r -= t;  // r' = (v - x²) - (2ax + a²)
            q += b;  // if x' = (x + a) then ax' = ax + a², thus q' = q' + b
        }
        b >>= 2;     // if a' = a/2, then b' = b / 4
    }
    return q;
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
 * Sets the clock for a given ADC
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
 * Set AFIO clock to a state
 */
void utilSetClockAFIO(FunctionalState state)
{
#ifdef RCC_APB1Periph_AFIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_AFIO,state);
#endif
#ifdef RCC_APB2Periph_AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,state);
#endif
#ifdef RCC_APB3Periph_AFIO
	RCC_APB3PeriphClockCmd(RCC_APB3Periph_AFIO,state);
#endif
#ifdef RCC_AHBPeriph_AFIO
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_AFIO,state);
#endif
}

/*
 * Inits a certain OC for a timer, based on a number
 */
void utilTIMOCInit(TIM_TypeDef* TIMx, uint8_t ch, TIM_OCInitTypeDef* TIM_OCInitStruct)
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
 * Sets the preload for the timer oc-channel
 */
void utilTIMOCConfigPreload(TIM_TypeDef* TIMx, uint8_t ch, uint16_t timOcPreload)
{
	switch(ch)
	{
	case 1:
		TIM_OC1PreloadConfig(TIMx,timOcPreload);
		break;
	case 2:
		TIM_OC2PreloadConfig(TIMx,timOcPreload);
		break;
	case 3:
		TIM_OC3PreloadConfig(TIMx,timOcPreload);
		break;
	case 4:
		TIM_OC4PreloadConfig(TIMx,timOcPreload);
		break;
	}
}
/*
 * Write a value to a timer channel, based on a number
 */
void utilTIMOCWrite(TIM_TypeDef* tim, uint8_t ch, uint32_t val)
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
