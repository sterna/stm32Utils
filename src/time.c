#include "time.h"

//Holds the number of ms since start
volatile uint32_t systemTime=0;

//Internal variables (init values assumes 72MHz). They are updated in timeInit
static uint32_t ticksPerUs=72;
static uint32_t ticksPerms=72000;

void SysTick_Handler(void)
{
	systemTime++;
}

void timeInit(void)
{
	SystemCoreClockUpdate();
	RCC_ClocksTypeDef RCC_Clocks;
	// SystTick configuration: an interrupt every 1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	if (SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 1000))
	{
			// Capture error
			while (1) {}
	}
	// Update the SysTick IRQ priority should be higher than others
	NVIC_SetPriority(SysTick_IRQn,0);
	ticksPerms=SystemCoreClock/1000;
	ticksPerUs=SystemCoreClock/1000000;
}

/*
 * delays the processor for a number of ms
 * during delay, the processor can only respond to interrupts
 */
/*
 * Delays for a number of ms
 */
void delay_ms(uint32_t ms)
{
	uint32_t end=ms+systemTime;
	while(systemTime<end){}
}

/*
 * Delays for a number of us
 * Might not be perfectly exact
 */
void delay_us(uint32_t us)
{
	asm volatile("nop");
	volatile uint32_t systickStartVal=SysTick->VAL;
	//If the start value is too large, it will get stuck
	if(systickStartVal>(ticksPerms-ticksPerUs))
	{
		systickStartVal=(ticksPerms-ticksPerUs);
	}
	else if(systickStartVal<ticksPerUs)
	{
		systickStartVal=ticksPerUs;
	}
	volatile uint32_t systickEndVal=0;
	while(us>1000)
	{
		while(SysTick->VAL<systickStartVal){}
		while(SysTick->VAL>systickStartVal){}
		us-=1000;
	}
	systickEndVal = SysTick->VAL-us*ticksPerUs;
	if(systickEndVal>ticksPerms)
	{
		systickEndVal+=ticksPerms;
		while(SysTick->VAL<systickEndVal){}
	}
	while(SysTick->VAL>systickEndVal){}
}

/*
 * Returns the number of microseconds since start
 * Will overflow in about 71 minutes
 */
uint32_t microSeconds()
{
	__disable_irq();
	uint32_t tickVal = SysTick->VAL;
	uint32_t systemTimeTemp = systemTime;
	__enable_irq();
	return systemTimeTemp*1000 + ((ticksPerUs*1000-tickVal)/ticksPerUs);
}

/*
 * Formats seconds to hours. minutes and seconds
 * Parameters are a pointer to the struct you want the value in and and a total of seconds
 */
void timeFormat(timeStruct* outStruct,unsigned long seconds)
{
	outStruct->hours=seconds/3600;
	outStruct->minutes=seconds/60-(outStruct->hours)*60;
	outStruct->seconds=seconds%60;
}
