/*HEADER******************************************************************************************
*
* Comments:
*
*
**END********************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Includes Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdlib.h>
#include "LPTimer.h"
#include "SW_Timer.h"


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Defines & Macros Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Maximum number of supported timers
#define SWTIMER_MAX_TIMERS	(16UL)
//! HW timer modulo
#define SWTIMER_HW_COUNTER	((OSCILLATOR/2)/1000UL)
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                       Typedef Section                        
///////////////////////////////////////////////////////////////////////////////////////////////////
//! SW timer element type
typedef struct
{
	uint32_t dwCounterReload;
	uint32_t dwCounter;
}SWTimer_t;
///////////////////////////////////////////////////////////////////////////////////////////////////
//                                  Function Prototypes Section                 
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Callbacks array
static void (* SWTimer_vfnpCallbacks[SWTIMER_MAX_TIMERS])(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Array with enable masks for each supported timer
const static uint16_t  SWTimer_gwaEnableMasks[SWTIMER_MAX_TIMERS] =
{
		(1<<0), (1<<1), (1<<2), (1<<3), (1<<4),
		(1<<5), (1<<6), (1<<7), (1<<8), (1<<9),
		(1<<10), (1<<11), (1<<12), (1<<13), (1<<14),
		(1<<15), 
};

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Constants Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Global Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
//                                   Static Variables Section                   
///////////////////////////////////////////////////////////////////////////////////////////////////
//! Holds the enabled timers. One per bit
static uint16_t SWTimer_gwTimersEnabled = 0;
//! SW Timer elements array
static SWTimer_t SWTimers_gtCounters[SWTIMER_MAX_TIMERS];
//! Amount of timers allocated
static uint8_t SWTimer_gbCurrrentTimers = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
//                                      Functions Section                       
///////////////////////////////////////////////////////////////////////////////////////////////////

void SWTimer_vfnInit(void)
{
	/* Initialize HW timer. Usually the time base is 1ms*/
	LPTimer_Init((uint8_t)LPTIMER_PRESCALER_BY_2,(uint8_t)LPTIMER_PRESCALER_CLOCK_3,SWTIMER_HW_COUNTER);
	
	LPTimer_EnableTimer();
}

uint8_t SWTimer_vfnAllocateChannel(uint32_t dwCounter, void (* vfnpTimerCallback)(void))
{
	uint8_t bStatus = SWTIMER_ERROR;
	/* Confirm there are timers available */
	if(SWTIMER_MAX_TIMERS > SWTimer_gbCurrrentTimers)
	{
		/* Load the counter and the reload value */
		SWTimers_gtCounters[SWTimer_gbCurrrentTimers].dwCounter = dwCounter;
		SWTimers_gtCounters[SWTimer_gbCurrrentTimers].dwCounterReload = dwCounter;
		
		/* Set the timer callback */
		if(vfnpTimerCallback != NULL)
		{
			SWTimer_vfnpCallbacks[SWTimer_gbCurrrentTimers] = vfnpTimerCallback;
		
			bStatus = SWTimer_gbCurrrentTimers;
			SWTimer_gbCurrrentTimers++;
			
		}
	}
	
	return(bStatus);
}

void SWTimer_vfnEnableTimer(uint8_t bTimerToEnable)
{	
	/* Set the proper timer bit */
	if(SWTIMER_MAX_TIMERS > bTimerToEnable)
	{
		SWTimer_gwTimersEnabled |= SWTimer_gwaEnableMasks[bTimerToEnable];
	}
}

void SWTimer_vfnUpdateCounter(uint8_t bTimerToUpdate, uint32_t dwNewCounter)
{
	/* Change the counter period */
	if(SWTIMER_MAX_TIMERS > bTimerToUpdate)
	{
		SWTimers_gtCounters[bTimerToUpdate].dwCounter = dwNewCounter;
		SWTimers_gtCounters[bTimerToUpdate].dwCounterReload = dwNewCounter;
	}
}

void SWTimer_vfnDisableTimer(uint8_t bTimerToDisable)
{
	/* Shutdown the timer */
	if(SWTIMER_MAX_TIMERS > bTimerToDisable)
	{
		SWTimer_gwTimersEnabled &= ~SWTimer_gwaEnableMasks[bTimerToDisable];
	}
}

void SWTimer_vfnServiceTimers(void)
{
	uint8_t bMaxCounter = SWTIMER_MAX_TIMERS - 1;
	
	/* Confirm there's a HW timer event */
	if(LPTIMER_CHECK_STATUS(LPTIMER_COUNTER_MATCH_MASK_STATUS))
	{
		LPTIMER_CLEAR_STATUS(LPTIMER_COUNTER_MATCH_MASK_STATUS);
		/* Update all timers and execute callbacks in case the period is met */
		do 
		{
			if(SWTimer_gwTimersEnabled&SWTimer_gwaEnableMasks[bMaxCounter])
			{
				SWTimers_gtCounters[bMaxCounter].dwCounter--;
				
				if(!SWTimers_gtCounters[bMaxCounter].dwCounter)
				{
					SWTimer_vfnpCallbacks[bMaxCounter]();
					SWTimers_gtCounters[bMaxCounter].dwCounter = SWTimers_gtCounters[bMaxCounter].dwCounterReload;
				}
			}
			
		}while(bMaxCounter--);
	}
}



///////////////////////////////////////////////////////////////////////////////////////////////////
// EOF
///////////////////////////////////////////////////////////////////////////////////////////////////
